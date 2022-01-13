import numpy as np
import copy
import sys
import cv2
import random
from scipy import ndimage
from Visualization import darp_area_visualization
import time
import random
import os
from numba import njit
np.set_printoptions(threshold=sys.maxsize)

random.seed(1)
os.environ['PYTHONHASHSEED'] = str(1)
np.random.seed(1)


@njit
def assign(droneNo, rows, cols, init_robot_pos, GridEnv, MetricMatrix, A):
    BWlist = np.zeros((droneNo, rows, cols))
    for r in range(droneNo):
        BWlist[r, init_robot_pos[r][0], init_robot_pos[r][1]] = 1

    ArrayOfElements = np.zeros(droneNo)
    for i in range(rows):
        for j in range(cols):
            if GridEnv[i, j] == -1:
                minV = MetricMatrix[0, i, j]
                indMin = 0
                for r in range(droneNo):
                    if MetricMatrix[r, i, j] < minV:
                        minV = MetricMatrix[r, i, j]
                        indMin = r

                A[i][j] = indMin
                BWlist[indMin, i, j] = 1
                ArrayOfElements[indMin] += 1

            elif GridEnv[i, j] == -2:
                A[i, j] = droneNo
    return BWlist, A, ArrayOfElements


@njit
def constructBinaryImages(A, val, rows, cols):
    BinaryRobot = np.copy(A)
    BinaryNonRobot = np.copy(A)
    for i in range(rows):
        for j in range(cols):
            if A[i, j] == 1:
                BinaryRobot[i, j] = 1
                BinaryNonRobot[i, j] = 0
            elif A[i, j] != 0:
                BinaryRobot[i, j] = 0
                BinaryNonRobot[i, j] = 1

    return BinaryRobot, BinaryNonRobot


@njit
def CalcConnectedMultiplier(rows, cols, dist1, dist2, CCvariation):
    returnM = np.zeros((rows, cols))
    MaxV = 0
    MinV = 2**30

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = dist1[i, j] - dist2[i, j]
            if MaxV < returnM[i, j]:
                MaxV = returnM[i, j]
            if MinV > returnM[i, j]:
                MinV = returnM[i, j]

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = (returnM[i, j]-MinV)*((2*CCvariation)/(MaxV - MinV)) + (1-CCvariation)

    return returnM


class DARP():
    def __init__(self, nx, ny, notEqualPortions, pos, portions, obs_pos,
                 visualization, MaxIter=80000, CCvariation=0.01,
                 randomLevel=0.0001, dcells=2,
                 importance=False):

        obstacles_positions = []
        initial_positions = []
        for position in pos:
            if position < 0 or position >= nx * ny:
                print("Initial positions should be inside the Grid.")
                sys.exit(2)
            initial_positions.append((position // ny, position % ny))

        for obstacle in obs_pos:
            if obstacle < 0 or obstacle >= nx * ny:
                print("Obstacles should be inside the Grid.")
                sys.exit(3)
            obstacles_positions.append((obstacle // ny, obstacle % ny))

        portions_new = []
        if notEqualPortions:
            for portion in portions:
                portions_new.append(portion)
        else:
            for drone in range(len(initial_positions)):
                portions_new.append(1 / len(initial_positions))

        portions = portions_new

        if len(initial_positions) != len(portions):
            print("Portions should be defined for each drone")
            sys.exit(4)

        s = sum(portions)
        if abs(s - 1) >= 0.0001:
            print("Sum of portions should be equal to 1.")
            sys.exit(1)

        for position in initial_positions:
            for obstacle in obstacles_positions:
                if position[0] == obstacle[0] and position[1] == obstacle[1]:
                    print("Initial positions should not be on obstacles")
                    sys.exit(3)

        print("\nInitial Conditions Defined:")
        print("Grid Dimensions:", nx, ny)
        print("Robot Number:", len(initial_positions))
        print("Initial Robots' positions", initial_positions)
        print("Portions for each Robot:", portions, "\n")

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

    def divideRegions(self):
        success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))

        while self.termThr <= self.dcells and not success and not cancelled:
            downThres = (self.Notiles - self.termThr*(self.droneNo-1))/(self.Notiles*self.droneNo)
            upperThres = (self.Notiles + self.termThr)/(self.Notiles*self.droneNo)

            success = True

            # Main optimization loop
            iteration = 0

            while iteration <= self.MaxIter and not cancelled:
                self.BWlist, self.A, self.ArrayOfElements = assign(self.droneNo,
                                                                   self.rows,
                                                                   self.cols,
                                                                   self.init_robot_pos,
                                                                   self.GridEnv,
                                                                   self.MetricMatrix,
                                                                   self.A)
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
                        BinaryRobot, BinaryNonRobot = constructBinaryImages(labels_im, r,self.rows,self.cols)
                        ConnectedMultiplier = CalcConnectedMultiplier(self.rows, self.cols,
                                                                      self.NormalizedEuclideanDistanceBinary(True, BinaryRobot, BinaryNonRobot),
                                                                      self.NormalizedEuclideanDistanceBinary(False, BinaryRobot, BinaryNonRobot),self.CCvariation)
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
        ind = np.where(self.A < self.droneNo)
        temp = (self.A[ind].astype(int),)+ind
        self.BinaryRobotRegions[temp] = True

    def generateRandomMatrix(self):
        RandomMa = np.zeros((self.rows, self.cols))
        randomlevel = 0.0001
        RandomMa = 2*randomlevel*np.random.uniform(0, 1,size=RandomMa.shape) + (1 - randomlevel)

        return RandomMa

    def FinalUpdateOnMetricMatrix(self, CM, RM, currentOne, CC):
        MMnew = np.zeros((self.rows, self.cols))
        MMnew = currentOne*CM*RM*CC

        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):
        for r in range(self.droneNo):
            if np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r]) > thresh or not connectedRobotRegions[r]:
                return False
        return True

    def update_connectivity(self):
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols))
        for i in range(self.droneNo):
            mask = np.where(self.A == i)
            self.connectivity[i, mask[0], mask[1]] = 255

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
        ind = np.where(self.GridEnv == -1)
        for (i, j) in zip(ind[0], ind[1]):
            indMin = np.argmin(self.MetricMatrix[:, i, j])
            self.A[i][j] = indMin
            self.BWlist[indMin, i, j] = 1
            self.ArrayOfElements[indMin] += 1
        ind = np.where(self.GridEnv == -2)
        self.A[ind] = self.droneNo

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
        if self.importance:
            if smallerthan0:
                returnCrit = (TilesImportance- MinimumImportance)*((correctionMult-1)/(MaximumImportance-MinimumImportance)) + 1
            else:
                returnCrit = (TilesImportance- MinimumImportance)*((1-correctionMult)/(MaximumImportance-MinimumImportance)) + correctionMult
        else:
            returnCrit[:, :] = correctionMult

        return returnCrit

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryRobot, BinaryNonRobot):
        if RobotR:
            distRobot = ndimage.morphology.distance_transform_edt(np.logical_not(BinaryRobot))
        else:
            distRobot = ndimage.morphology.distance_transform_edt(np.logical_not(BinaryNonRobot))

        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        #Normalization
        if RobotR:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV)) + 1
        else:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV))

        return distRobot

