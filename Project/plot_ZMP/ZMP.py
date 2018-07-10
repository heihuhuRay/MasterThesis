import matplotlib.pyplot as plt

def cal_zmp(L_fsr_fl, L_fsr_fr, L_fsr_rl, L_fsr_rr, R_fsr_fl, R_fsr_fr, R_fsr_rl, R_fsr_rr):
    x= 0
    y= 1

    LFL = [0.07025, 0.0299]
    LFR = [0.07025,	-0.0231]
    LRL = [-0.03025, 0.0299]
    LRR = [-0.02965, -0.0191]
    sumLx = LFL[x] + LFR[x] + LRL[x] + LRR[x]
    sumLy = LFL[y] + LFR[y] + LRL[y] + LRR[y]

    RFL = [0.07025, 0.0231]
    RFR	= [0.07025, -0.0299]
    RRL	= [-0.03025, 0.0191]
    RRR = [-0.02965, -0.0299]
    sumRx = RFL[x] + RFR[x] + RRL[x] + RRR[x]
    sumRy = RFL[y] + RFR[y] + RRL[y] + RRR[y]

    LeftX = (LFL[x]*L_fsr_fl + LFR[x]*L_fsr_fr + LRL[x]*L_fsr_rl + LRR[x]*L_fsr_rr)/sumLx
    LeftY = (LFL[y]*L_fsr_fl + LFR[y]*L_fsr_fr + LRL[y]*L_fsr_rl + LRR[y]*L_fsr_rr)/sumLy

    zmpLeft = [LeftX, LeftY]

    RightX = (RFL[x]*R_fsr_fl + RFR[x]*R_fsr_fr + RRL[x]*R_fsr_rl + RRR[x]*R_fsr_rr)/sumRx
    RightY = (RFL[y]*R_fsr_fl + RFR[y]*R_fsr_fr + RRL[y]*R_fsr_rl + RRR[y]*R_fsr_rr)/sumRy

    zmpRight = [RightX, RightY]

    return [zmpLeft, zmpRight]

def plotZMP(zmp):
    x = 0
    y = 1

    LFL = [0.07025, 0.0299]
    LFR = [0.07025, -0.0231]
    LRL = [-0.03025, 0.0299]
    LRR = [-0.02965, -0.0191]
    leftLegFSRCordX = [LFL[x], LFR[x], LRR[x], LRL[x], LFL[x]]
    leftLegFSRCordY = [LFL[y], LFR[y], LRR[y], LRL[y], LFL[y]]

    RFL = [0.07025, 0.0231]
    RFR = [0.07025, -0.0299]
    RRL = [-0.03025, 0.0191]
    RRR = [-0.02965, -0.0299]
    rightLegFSRCordX = [RFL[x], RFR[x], RRR[x], RRL[x], RFL[x]]
    rightLegFSRCordY = [RFL[y], RFR[y], RRR[y], RRL[y], RFL[y]]

    #data2print1 = [zmp[0][0], zmp[1][0]]
    #data2print2 = [zmp[0][1], zmp[1][1]]

    zmpRight = zmp[1]
    zmpLeft = zmp[0]

    plt.figure(1)
    plt.subplot(121)
    plt.plot(rightLegFSRCordX, rightLegFSRCordY,'ro-', zmpRight[0], zmpRight[1], 'go')
    plt.title("Right Leg")

    plt.subplot(122)
    plt.plot(leftLegFSRCordX, leftLegFSRCordY, 'ro-', zmpLeft[0], zmpLeft[1], 'go')
    plt.title("Left Leg")

    #plt.grid(True)
    plt.show()