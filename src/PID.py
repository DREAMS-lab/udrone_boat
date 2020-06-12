from collections import deque


class PID:
    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.error_buffer = deque(maxlen=50)

    def run(self, actual, desired):
        error = actual - desired
        integral = error
        for prev_error in self.error_buffer:
            integral += prev_error
        if self.error_buffer:
            e = self.error_buffer.pop()  # pop
        else:
            e = 0.0
        derivative = error - e
        self.error_buffer.appendleft(e)  # insert back
        self.error_buffer.appendleft(error)
        return self.kp*error + self.ki*integral + self.kd*derivative
