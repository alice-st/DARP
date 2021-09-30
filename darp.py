import numpy as np
import copy
import sys
import cv2
import random
from scipy import ndimage
from Visualization import darp_area_visualization
import time

np.set_printoptions(threshold=sys.maxsize)


class DARP():
    def __init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions, visualization):
        self.rows = nx
        self.cols = ny
        self.visualization = visualization
        empty_space = []
        if nx > ny:
            for j in range(ny, nx):
                for i in range(nx):
                    empty_space.append((i, j))
            self.cols = self.rows
        elif ny > nx:
            for j in range(nx, ny):
                for i in range(ny):
                    empty_space.append((j, i))
            self.rows = self.cols

        self.GridEnv = np.full(shape=(self.rows, self.cols), fill_value=0)

        for cell in empty_space:
            self.GridEnv[cell[0], cell[1]] = 1

        if obstacles_positions != []:
            for obstacle in obstacles_positions:
                self.GridEnv[obstacle[0], obstacle[1]] = 1
        for pos in initial_positions:
            self.GridEnv[pos[0], pos[1]] = 2

        print("Given Grid area:")
        print(self.GridEnv)

        self.droneNo = 0
        self.A = np.zeros((self.rows, self.cols))
        self.init_robot_pos = []
        self.ob = 0
        self.defineRobotsObstacles()
        self.MaxIter = MaxIter
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.notEqualPortions = notEqualPortions
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols))
        self.BinaryRobotRegions = np.zeros((self.droneNo, self.rows, self.cols), dtype=bool)

        # If user has not defined custom portions divide area equally for all drones
        self.Rportions = np.zeros((self.droneNo))
        if (not notEqualPortions):
            for i in range(self.droneNo):
                self.Rportions[i] = 1.0/self.droneNo
        else:
            for i in range(self.droneNo):
                self.Rportions[i] = portions[i]

        self.AllDistances, self.termThr, self.Notiles, self.DesireableAssign, self.TilesImportance, self.MinimumImportance, self.MaximumImportance= self.construct_Assignment_Matrix()
        self.MetricMatrix = copy.deepcopy(self.AllDistances)
        self.BWlist = np.zeros((self.droneNo, self.rows, self.cols))
        self.ArrayOfElements = np.zeros(self.droneNo)
        self.color = []

        for r in range(self.droneNo):
            self.color.append(list(np.random.choice(range(256), size=3)))

        if self.visualization:
            self.assignment_matrix_visualization = darp_area_visualization(self.A, self.droneNo, self.color)

        self.success = self.update()

    def defineRobotsObstacles(self):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.GridEnv[i, j] == 2:
                    self.GridEnv[i, j] = self.droneNo
                    self.A[i, j] = self.droneNo
                    self.droneNo += 1
                    self.init_robot_pos.append((i, j))
                elif(self.GridEnv[i, j] == 1):
                    self.ob += 1
                    self.GridEnv[i, j] = -2
                else:
                    self.GridEnv[i, j] = -1

    def update(self):
        success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))

        while self.termThr <= self.dcells and not success and not cancelled:
            downThres = (self.Notiles - self.termThr*(self.droneNo-1))/(self.Notiles*self.droneNo)
            upperThres = (self.Notiles + self.termThr)/(self.Notiles*self.droneNo)

            success = True

            #main optimization loop
            iteration = 0

            while iteration <= self.MaxIter and not cancelled:
                self.assign()
                ConnectedMultiplierList = np.ones((self.droneNo, self.rows, self.cols))
                ConnectedRobotRegions = np.zeros(self.droneNo)
                plainErrors = np.zeros((self.droneNo))
                divFairError = np.zeros((self.droneNo))

                for r in range(self.droneNo):
                    ConnectedMultiplier = np.ones((self.rows, self.cols))
                    ConnectedRobotRegions[r] = True
                    self.update_connectivity()
                    image = np.uint8(self.connectivity[r, :, :])
                    num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)
                    if num_labels > 2:
                        ConnectedRobotRegions[r] = False
                        BinaryRobot, BinaryNonRobot = self.constructBinaryImages(labels_im, r)
                        ConnectedMultiplier = self.CalcConnectedMultiplier(
                            self.NormalizedEuclideanDistanceBinary(True, BinaryRobot, BinaryNonRobot),
                            self.NormalizedEuclideanDistanceBinary(False, BinaryRobot, BinaryNonRobot))
                    ConnectedMultiplierList[r, :, :] = ConnectedMultiplier
                    plainErrors[r] = self.ArrayOfElements[r]/(self.DesireableAssign[r]*self.droneNo)
                    if plainErrors[r] < downThres:
                        divFairError[r] = downThres - plainErrors[r]
                    elif plainErrors[r] > upperThres:
                        divFairError[r] = upperThres - plainErrors[r]

                if self.IsThisAGoalState(self.termThr, ConnectedRobotRegions):
                    print("\nFinal Assignment Matrix:")
                    print(self.A)
                    break

                TotalNegPerc = 0
                totalNegPlainErrors = 0
                correctionMult = np.zeros(self.droneNo)

                for r in range(self.droneNo):
                    if divFairError[r] < 0:
                        TotalNegPerc += np.absolute(divFairError[r])
                        totalNegPlainErrors += plainErrors[r]

                    correctionMult[r] = 1

                for r in range(self.droneNo):
                    if totalNegPlainErrors != 0:
                        if divFairError[r] < 0:
                            correctionMult[r] = 1 + (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)
                        else:
                            correctionMult[r] = 1 - (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)

                        criterionMatrix = self.calculateCriterionMatrix(
                                self.TilesImportance[r],
                                self.MinimumImportance[r],
                                self.MaximumImportance[r],
                                correctionMult[r],
                                divFairError[r] < 0)

                    self.MetricMatrix[r] = self.FinalUpdateOnMetricMatrix(
                            criterionMatrix,
                            self.generateRandomMatrix(),
                            self.MetricMatrix[r],
                            ConnectedMultiplierList[r, :, :])

                iteration += 1
                if self.visualization:
                    self.assignment_matrix_visualization.placeCells(self.A)
                # time.sleep(0.5)

            if iteration >= self.MaxIter:
                self.MaxIter = self.MaxIter/2
                success = False
                self.termThr += 1

        self.getBinaryRobotRegions()
        return success

    def getBinaryRobotRegions(self):

        for i in range(self.rows):
            for j in range(self.cols):
                if self.A[i][j] < self.droneNo:
                    self.BinaryRobotRegions[int(self.A[i, j]), i, j] = True
                # else:
                #     self.BinaryRobotRegions[int(self.A[i, j]), i, j] = False

    def generateRandomMatrix(self):
        RandomMa = np.zeros((self.rows, self.cols))
        randomlevel = 0.0001
        for i in range(self.rows):
            for j in range(self.cols):
                RandomMa[i][j] = 2*randomlevel*random.uniform(0, 1) + (1 - randomlevel)

        return RandomMa

    def FinalUpdateOnMetricMatrix(self, CM, RM, currentOne, CC):
        MMnew = np.zeros((self.rows, self.cols))

        for i in range(self.rows):
            for j in range(self.cols):
                MMnew[i, j] = currentOne[i, j]*CM[i, j]*RM[i, j]*CC[i, j]

        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):
        for r in range(self.droneNo):
            #TODO if thresh == 0?
            if np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r]) > thresh or not connectedRobotRegions[r]:
                return False
        return True

    def update_connectivity(self):
        for i in range(self.droneNo):
            for j in range(self.rows):
                for k in range(self.cols):
                    if self.A[j, k] == i:
                        self.connectivity[i, j, k] = 255
                    else:
                        self.connectivity[i, j, k] = 0

    def constructBinaryImages(self, A, val):
        BinaryRobot = np.copy(A)
        BinaryNonRobot = np.copy(A)
        for i in range(self.rows):
            for j in range(self.cols):
                if A[i, j] == 1:
                    BinaryRobot[i, j] = 1
                    BinaryNonRobot[i, j] = 0
                elif A[i, j] != 0:
                    BinaryRobot[i, j] = 0
                    BinaryNonRobot[i, j] = 1

        return BinaryRobot, BinaryNonRobot

    def assign(self):
        self.BWlist = np.zeros((self.droneNo, self.rows, self.cols))
        for r in range(self.droneNo):
            self.BWlist[r, self.init_robot_pos[r][0], self.init_robot_pos[r][1]] = 1

        self.ArrayOfElements = np.zeros(self.droneNo)
        for i in range(self.rows):
            for j in range(self.cols):
                if self.GridEnv[i, j] == -1:
                    minV = self.MetricMatrix[0, i, j]
                    indMin = 0
                    for r in range(self.droneNo):
                        if self.MetricMatrix[r, i, j] < minV:
                            minV = self.MetricMatrix[r, i, j]
                            indMin = r

                    self.A[i][j] = indMin
                    self.BWlist[indMin, i, j] = 1
                    self.ArrayOfElements[indMin] += 1

                elif self.GridEnv[i, j] == -2:
                    self.A[i, j] = self.droneNo

    # Construct Assignment Matrix
    def construct_Assignment_Matrix(self):
        Notiles = self.rows*self.cols
        fair_division = 1/self.droneNo
        effectiveSize = Notiles - self.droneNo - self.ob
        termThr = 0

        if effectiveSize % self.droneNo != 0:
            termThr = 1

        DesireableAssign = np.zeros(self.droneNo)
        MaximunDist = np.zeros(self.droneNo)
        MaximumImportance = np.zeros(self.droneNo)
        MinimumImportance = np.zeros(self.droneNo)

        for i in range(self.droneNo):
            DesireableAssign[i] = effectiveSize * self.Rportions[i]
            MinimumImportance[i] = sys.float_info.max
            if (DesireableAssign[i] != int(DesireableAssign[i]) and termThr != 1):
                termThr = 1

        AllDistances = np.zeros((self.droneNo, self.rows, self.cols))
        TilesImportance = np.zeros((self.droneNo, self.rows, self.cols))

        for x in range(self.rows):
            for y in range(self.cols):
                tempSum = 0
                for r in range(self.droneNo):
                    AllDistances[r, x, y] = np.linalg.norm(np.array(self.init_robot_pos[r]) - np.array((x, y)))  # E!
                    if AllDistances[r, x, y] > MaximunDist[r]:
                        MaximunDist[r] = AllDistances[r, x, y]
                    tempSum += AllDistances[r, x, y]

                for r in range(self.droneNo):
                    if tempSum - AllDistances[r, x, y] != 0:
                        TilesImportance[r, x, y] = 1/(tempSum - AllDistances[r, x, y])
                    else:
                        TilesImportance[r, x, y] = 1
                    # Todo FixMe!
                    if TilesImportance[r, x, y] > MaximumImportance[r]:
                        MaximumImportance[r] = TilesImportance[r, x, y]

                    if TilesImportance[r, x, y] < MinimumImportance[r]:
                        MinimumImportance[r] = TilesImportance[r, x, y]

        return AllDistances, termThr, Notiles, DesireableAssign, TilesImportance, MinimumImportance, MaximumImportance

    def calculateCriterionMatrix(self, TilesImportance, MinimumImportance, MaximumImportance, correctionMult, smallerthan0,):
        returnCrit = np.zeros((self.rows, self.cols))
        for i in range(self.rows):
            for j in range(self.cols):
                if self.importance:
                    if smallerthan0:
                        returnCrit[i, j] = (TilesImportance[i, j] - MinimumImportance)*((correctionMult-1)/(MaximumImportance-MinimumImportance)) + 1
                    else:
                        returnCrit[i, j] = (TilesImportance[i, j] - MinimumImportance)*((1-correctionMult)/(MaximumImportance-MinimumImportance)) + correctionMult
                else:
                    returnCrit[i, j] = correctionMult

        return returnCrit

    def CalcConnectedMultiplier(self, dist1, dist2):
        returnM = np.zeros((self.rows, self.cols))
        MaxV = 0
        MinV = sys.float_info.max

        for i in range(self.rows):
            for j in range(self.cols):
                returnM[i, j] = dist1[i, j] - dist2[i, j]
                if MaxV < returnM[i, j]:
                    MaxV = returnM[i, j]
                if MinV > returnM[i, j]:
                    MinV = returnM[i, j]

        for i in range(self.rows):
            for j in range(self.cols):
                returnM[i, j] = (returnM[i, j]-MinV)*((2*self.CCvariation)/(MaxV - MinV)) + (1-self.CCvariation)

        return returnM

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryRobot, BinaryNonRobot):
        if RobotR:
            distRobot = ndimage.morphology.distance_transform_edt(np.logical_not(BinaryRobot))
        else:
            distRobot = ndimage.morphology.distance_transform_edt(np.logical_not(BinaryNonRobot))

        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        #Normalization
        for i in range(self.rows):
            for j in range(self.cols):
                if RobotR:
                    distRobot[i, j] = (distRobot[i, j] - MinV)*(1/(MaxV-MinV)) + 1
                else:
                    distRobot[i, j] = (distRobot[i, j] - MinV)*(1/(MaxV-MinV))

        return distRobot
