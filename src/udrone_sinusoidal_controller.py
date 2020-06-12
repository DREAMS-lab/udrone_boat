import rospy
from geometry_msgs.msg import Quaternion, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler
import tf
from PID import PID
import numpy as np
from udrone_boat.msg import UdroneStates


class UdroneController:
    """ Attitude controller for PX4-UAV offboard mode """

    def __init__(self):
        self.mode = "ATTITUDE"
        self.arm = False
        self.udrone_state = UdroneStates()

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.listener = tf.TransformListener()
        self.state_sub = rospy.Subscriber('/udrone/mavros/state', State, callback=self.state_callback)
        self.att_pub = rospy.Publisher('/udrone/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.decision = rospy.Subscriber('/data', String, callback=self.set_mode)
        rospy.Subscriber("udroneboat", UdroneStates, self.set_udrone_state)
        self.att = AttitudeTarget()
        self.controller()

    def set_udrone_state(self, msg):
        self.udrone_state = msg

    def set_mode(self, msg):
        self.mode = str(msg.data)

    def state_callback(self, msg):
        if msg.mode == 'OFFBOARD' and self.arm == True:
            pass
        else:
            self.take_off()

    def set_offboard_mode(self):
        rospy.wait_for_service('/udrone/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/udrone/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm(self):
        rospy.wait_for_service('/udrone/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/udrone/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def take_off(self):
        self.set_offboard_mode()
        self.set_arm()

    def send_att(self):
        rate = rospy.Rate(100)  # Hz
        self.att.body_rate = Vector3(0, 0, 2)
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.att.thrust = 0.3
        self.att.type_mask = 7  # ignore body rate

        xPID = PID(kp=0.2, kd=0.2, ki=0.1)
        zPID = PID(kp=0.03, kd=0.01, ki=0.01)
        yPID = PID(kp=0.03, kd=0.01, ki=0.01)

        time = np.arange(0, 2*np.pi, 0.001)
        amplitude = 2 * np.sin(time)
        length = amplitude.shape[0]
        i = 0
        while not rospy.is_shutdown():
            try:
                trans = (self.udrone_state.x, self.udrone_state.y, self.udrone_state.z)
                print("Estimate: ", trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as Ex:
                print(Ex)
                self.att.header.stamp = rospy.Time.now()
                self.att.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
                self.att.thrust = 0.3
                self.att_pub.publish(self.att)
                rate.sleep()
                continue

            x_error = xPID.run(trans[0], 0)
            y_error = yPID.run(trans[1], amplitude[i])
            i += 1
            i = i % length
            z_error = zPID.run(trans[2], 5)

            # if z_error is positive, udrone is far from boat
            # so increase pitch to increase the altitude
            if z_error < -1.5709:
                z_error = -1.5709
            elif z_error > 1.5709:
                z_error = 1.5709

            # if y_error is positive, udrone is far from boat
            # so increase pitch to increase the altitude
            if y_error < -1.5709:
                y_error = -1.5709
            elif y_error > 1.5709:
                y_error = 1.5709

            if x_error < 0:
                x_error = 0.0
            elif x_error > 1:
                x_error = 1.0

            print(x_error, y_error, z_error)
            self.att.header.stamp = rospy.Time.now()
            self.att.orientation = Quaternion(*quaternion_from_euler(0, -1*z_error, y_error))
            self.att.thrust = x_error
            self.att_pub.publish(self.att)
            rate.sleep()

    def controller(self):
        """ A state machine developed to convert UAV movement to fixed states """
        while not rospy.is_shutdown():
            if self.mode == "ATTITUDE":
                self.send_att()


if __name__ == "__main__":
    UdroneController()
