class KalmanFilter:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0;
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def setAngle(self, newAngle):
        self.angle = newAngle

    def calcAngle(self, newAngle, newRate, dt):
        Q_angle = 0.001
        Q_bias = 0.003
        R_measure = 0.03

        # step1
        rate = newRate - self.bias
        self.angle += dt * rate

        # step2
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += Q_bias * dt

        # step3
        y = newAngle - self.angle

        # step4
        S = self.P[0][0] + R_measure

        # step5
        K = [self.P[0][0] / S, self.P[1][0]]

        # step6
        self.angle += K[0] * y
        self.bias += K[1] * y

        # step7
        p00_temp = self.P[0][0]
        p01_temp = self.P[0][1]
        self.P[0][0] -= K[0] * p00_temp
        self.P[0][1] -= K[0] * p01_temp
        self.P[1][0] -= K[1] * p00_temp
        self.P[1][1] -= K[1] * p01_temp

        return self.angle

    