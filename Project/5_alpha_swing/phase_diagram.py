import matplotlib.pyplot as plt
import NaoConnect


# read the orientaion of the robot
#(FsrLeft, FsrRight, robPos, robOrient, HeadTouch, HandTouchLeft, HandTouchRight) = NaoConnect.NaoGetSensors()
# robo_orientaion_x.append(robOrient[0])
# robo_orientaion_y.append(robOrient[1])
# robo_orientaion_z.append(robOrient[2])

# calculate and plot derivatives of the all orientaion

# cal_derivities(robo_orientaion_x, 0.015)
# cal_derivities(robo_orientaion_y, 0.015)
# cal_derivities(robo_orientaion_z, 0.015)


# function to calculate derivatives
def cal_derivities(theta, dt):
    dev_output = []
    for i in range(0, len(theta)-1):
        dev_output.append((theta[i+1] - theta[i])/dt)
    theta.pop()
    plt.figure(1)
    plt.plot(theta, dev_output, 'b')

    plt.xlabel('theta')
    plt.ylabel('Darivated theta')
    plt.grid(True)

    plt.show()
    return  dev_output