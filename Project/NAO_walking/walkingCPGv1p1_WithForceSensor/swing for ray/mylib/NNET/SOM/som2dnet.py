import math
import sys
import numpy as np
from matplotlib import pyplot as mp
import pylab as pl


class ParamVar(object):
    def __init__(self):
        self.Param = ParamVarInit()


# this class is for the associated parameters to each node, the coordination parameters for drawing Lines, Circle & Arc
class ParamVarInit(object):
    LineParam = np.zeros([8, 2])
    CircleParam = np.ones(8)
    CurveParam = np.zeros([16,4])

class SOM_Class1:
    def __init__(self, dimensionInput, maxClusters, ClustersDim1, alphaStart, minimumAlpha, decayRate):

        self.dimIn = dimensionInput
        self.mAlpha = alphaStart # it will be reduced in train function 
        self.Alpha = alphaStart
        self.minAlpha = minimumAlpha
        self.decayRate = decayRate
        self.mIterations = 0
        self.maxClusters = maxClusters
        self.ClustersLength = ClustersDim1
        self.ClustersWidht = self.maxClusters / self.ClustersLength
        self.mD = [[]] * maxClusters
        self.w = np.random.rand(self.maxClusters,self.dimIn)*1.0
        self.nodeInfo = [[]] * maxClusters
        for ii in range(0,maxClusters):
            self.nodeInfo[ii] = ParamVar()
        self.flagNode = np.zeros(maxClusters)
        self.winnerNodeIndx = []
        self.trainNodeIndex = []
        print self.w
        return

    def compute_input(self, vectorNumber, trainingTests):


        for i in range(self.maxClusters):
            self.mD[i] = 0.0
            for j in range(self.dimIn):
                self.mD[i] += math.pow((self.w[i][j] - trainingTests[vectorNumber][j]), 2)

        return

    def train(self, patterns,PlotIsON='False'):
        self.flagNode = np.zeros(len(self.flagNode))
        self.trainNodeIndex = []
        self.mIterations = 0
        flag_while = 0
        while self.mAlpha > self.minAlpha:
            if (self.mIterations%50==0):
                print "Training Iterations= " ,self.mIterations

            self.mIterations += 1
            for i in range(len(patterns)):
                self.compute_input(i, patterns)

                dMin = np.argmin(self.mD)
                # Update the weights on the winning unit.
                #for j in range(self.mVectors):
                self.w[dMin][:] +=  (self.mAlpha * (patterns[i][:] - self.w[dMin][:]))

                # to keep a track to see which nodes with the set of training samples are activated:
                self.winnerNodeIndx = dMin
                if (flag_while == 0):
                    self.flagNode[dMin] = 1
                    self.trainNodeIndex = np.append(self.trainNodeIndex, self.winnerNodeIndx)
                # Update the weights on the neighbors of the winning unit.
                # first neighbours 
                first_neighboursMult = 0.65
                if ((dMin+1)in range(0, self.maxClusters))and(((dMin+1)%self.ClustersLength) != 0):
                    self.w[dMin+1][:] +=  (self.mAlpha * (patterns[i][:] - self.w[dMin+1][:]))*first_neighboursMult

                if (dMin-1)in range(0, self.maxClusters)and(((dMin)%self.ClustersLength) != 0):
                    self.w[dMin-1][:] +=  (self.mAlpha * (patterns[i][:] - self.w[dMin-1][:]))*first_neighboursMult

                if (dMin+self.ClustersLength)in range(0, self.maxClusters):
                    self.w[dMin+self.ClustersLength][:] +=  (self.mAlpha * (patterns[i][:] - self.w[dMin+self.ClustersLength][:]))*first_neighboursMult

                if (dMin-self.ClustersLength)in range(0, self.maxClusters):
                    self.w[dMin-self.ClustersLength][:] +=  (self.mAlpha * (patterns[i][:] - self.w[dMin-self.ClustersLength][:]))*first_neighboursMult

            flag_while = 1



            # Reduce the learning rate.
            self.mAlpha = self.decayRate * self.mAlpha

            if (PlotIsON =="True"):
                self.plot2D(patterns)
        self.mAlpha = self.Alpha
        return


    def test(self, patterns):
        # Print clusters created.
        sys.stdout.write("Clusters for input:\n")

        for i in range(len(patterns)):
            self.compute_input(i, patterns)

            dMin = np.argmin(self.mD)

            sys.stdout.write("\nVector ( ")

            for j in range(self.dimIn):
                sys.stdout.write(str(patterns[i][j]) + ", ")

            sys.stdout.write(") fits into category " + str(dMin) + "\n")

        return


    def get_iterations(self):
        return self.mIterations

    def plot2D(self, trainingset=[]):

        pl.figure('SOM plot 2D')
        pl.clf()
        if len(trainingset)>0:
            pl.plot(trainingset[0:len(trainingset),0],trainingset[0:len(trainingset),1], '.')
        for i in range(self.maxClusters):
            pl.plot(self.w[i,0], self.w[i,1], 'go')
            if ((i+1)in range(0, self.maxClusters))and(((i+1)%self.ClustersLength) != 0):
                pl.plot([self.w[i][0] , self.w[i+1][0]],[self.w[i][1] , self.w[i+1][1]], 'r')

            if (i-1)in range(0, self.maxClusters)and(((i)%self.ClustersLength) != 0):
                pl.plot([self.w[i][0] , self.w[i-1][0]],[self.w[i][1] , self.w[i-1][1]], 'r')

            if (i+self.ClustersLength)in range(0, self.maxClusters):
                pl.plot([self.w[i][0] , self.w[i+self.ClustersLength][0]],[self.w[i][1] , self.w[i+self.ClustersLength][1]], 'r')

            if (i-self.ClustersLength)in range(0, self.maxClusters):
                pl.plot([self.w[i][0] , self.w[i-self.ClustersLength][0]],[self.w[i][1] , self.w[i-self.ClustersLength][1]], 'r')

        #pl.show(block=False)
        pl.show()
        pl.pause(0.00001)
        return
        


