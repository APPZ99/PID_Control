class PID_Controller_c:

    def __init__(self, KP = 0.0, KI = 0.0, KD = 0.0, maxOutput = 0.0, maxIOutput = 0.0):

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.error = [0.0, 0.0, 0.0]
        self.diff = [0.0, 0.0, 0.0]

        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0

        self.output = 0.0
        self.maxOutput = maxOutput
        self.maxIOutput = maxIOutput

    def ClearPID(self):

        self.KP = 0.0
        self.KI = 0.0
        self.KD = 0.0

        self.error = [0.0, 0.0, 0.0]
        self.diff = [0.0, 0.0, 0.0]

        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0

    def LimitOutput(self, output, limitOutput):

        if(output > limitOutput):
            output = limitOutput
        elif(output < -limitOutput):
            output = -limitOutput

        return output

    def PositionMode(self, feedback, target):

        self.feedback = feedback
        self.target = target
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = target - feedback

        self.Pout = self.KP * self.error[0]
        self.Iout += self.KI * self.error[0]
        self.diff[2] = self.diff[1]
        self.diff[1] = self.diff[0]
        self.diff[0] = (self.error[0] - self.error[1])
        self.Dout = self.KD * self.diff[0]
        self.Iout = self.LimitOutput(self.Iout, self.maxIOutput)
        self.output = self.Pout + self.Iout + self.Dout
        self.output = self.LimitOutput(self.output, self.maxOutput)
        
        return self.output 

    def IncrementMode(self, feedback, target):

        self.feedback = feedback
        self.target = target
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = target - feedback

        self.Pout = self.KP * (self.error[0] - self.error[1])
        self.Iout = self.KI * self.error[0]
        self.diff[2] = self.diff[1]
        self.diff[1] = self.diff[0]
        self.diff[0] = (self.error[0] - 2 * self.error[1] + self.error[2])
        self.Dout = self.KD * self.diff[0]
        self.output += self.Pout + self.Iout + self.Dout
        self.output = self.LimitOutput(self.output, self.maxOutput)

        return self.output