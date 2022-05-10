'''
@ author: APPZ99
@ description: A PID controller based on python

'''

import numpy as np
import matplotlib.pyplot as plt
from PID_Control import PID_Controller_c as PID
from scipy.interpolate import BSpline, make_interp_spline

def drawPID():

    KP = 1.1
    KI = 0.001
    KD = 0.0
    maxOutput = 3
    maxIOutput = 0.5

    loopNum = 50
    target = 0.0
    feedback = 0.0


    pidController = PID(KP, KI, KD, maxOutput, maxIOutput)
    
    feedbackList = []
    timeList = []
    targetList = []

    for i in range(loopNum):
        pidController.PositionMode(feedback, target)
        output = pidController.output

        if target > 0:
            feedback += (output - (1/i))

        if(i > 15):
            target = 2
        '''
        if(i > 50):
            target = 4
        if(i > 75):
            target = 8
        '''
        feedbackList.append(feedback)
        targetList.append(target)
        timeList.append(i)


    timeSM = np.array(timeList)
    # 在指定的间隔内返回均匀间隔的数字
    timeSmooth = np.linspace(timeSM.min(), timeSM.max(), 300)
    # 拟合一条曲线 - 拟合后，曲线才能绘制出来
    feedbackSmooth = make_interp_spline(timeList, feedbackList)(timeSmooth)
    
    plt.plot(timeSmooth, feedbackSmooth, color='b')
    plt.plot(timeList, targetList, color='r')
    plt.xlim((0, loopNum))
    plt.ylim((min(feedbackList)-0.5, max(feedbackList)+0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID')
    plt.title('TEST PID')

    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    drawPID()



