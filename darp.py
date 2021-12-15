import numpy as np
import copy
import sys
import cv2
from scipy import ndimage
from Visualization import darp_area_visualization

np.set_printoptions(threshold=sys.maxsize)


class DARP:
    def __init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions,
                 initial_positions, portions, obstacles_positions, visualization):
        self.rows = nx
        self.cols = ny
        self.effectiveSize = 0
        self.visualization = visualization
        empty_space = []

        # TODO fix this input to be flexible in all dimensions
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

        self.A = np.zeros((self.rows, self.cols))
        self.ob = 0
        self.GridEnv = self.defineGridEnv(initial_positions, obstacles_positions, empty_space)
        # print("Given Grid area:")
        # print(self.GridEnv)

        self.init_robot_pos = initial_positions
        self.MaxIter = MaxIter
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.notEqualPortions = notEqualPortions
        self.connectivity = np.zeros((len(self.init_robot_pos), self.rows, self.cols))
        self.BinaryRobotRegions = np.zeros((len(self.init_robot_pos), self.rows, self.cols), dtype=bool)

        # If user has not defined custom portions divide area equally for all drones
        self.Rportions = np.zeros(len(self.init_robot_pos))
        if notEqualPortions:
            if len(self.init_robot_pos) != len(portions):
                print("notEqualPortions was set to True, but portions number and initial robot number don't match")
                sys.exit(5)
            else:
                for idx, robot in enumerate(self.init_robot_pos):
                    self.Rportions[idx] = portions[idx]
        else:
            for idx, robot in enumerate(self.init_robot_pos):
                self.Rportions[idx] = 1.0 / len(self.init_robot_pos)

        self.AllDistances, self.termThr, self.Notiles, self.DesireableAssign, self.TilesImportance, self.MinimumImportance, self.MaximumImportance = self.construct_Assignment_Matrix()
        self.MetricMatrix = copy.deepcopy(self.AllDistances)
        self.BWlist = np.zeros((len(self.init_robot_pos), self.rows, self.cols))
        self.ArrayOfElements = np.zeros(len(self.init_robot_pos))

        self.color = []
        for robot in self.init_robot_pos:
            self.color.append(list(np.random.choice(range(256), size=3)))

        if self.visualization:
            self.assignment_matrix_visualization = darp_area_visualization(self.A, len(self.init_robot_pos), self.color, self.init_robot_pos)
        self.success = self.update()

    def defineGridEnv(self, init_robot_pos, obstacles_positions, empty_space):
        """
        Defines and returns the GridEnv(iroment) array for later use.
        All tiles except obstacles, empty_space, initial robo start points will have the value -1.
        :param init_robot_pos: The initial robot start points (array) - tile value will be their array.index number
        :param obstacles_positions: The given array of obstacle tiles. obstacle tile value is -2
        :param empty_space: The area/tiles positions (array) which are defined like obstacle tiles
        :return: The prepared GridEnv
        """

        local_grid_env = np.full(shape=(self.rows, self.cols), fill_value=-1)  # create non obstacle map with value -1

        # initial robot tiles will have their array.index as value
        for idx, robot in enumerate(init_robot_pos):
            local_grid_env[robot] = idx
            self.A[robot] = idx

        # obstacle tiles value is -2
        self.ob = len(obstacles_positions)
        for idx, obstacle_pos in enumerate(obstacles_positions):
            local_grid_env[obstacle_pos[0], obstacle_pos[1]] = -2
        for idx, es_pos in enumerate(empty_space):
            local_grid_env[es_pos] = -2

        return local_grid_env

    def update(self):
        success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))

        while self.termThr <= self.dcells and not success and not cancelled:
            downThres = (self.Notiles - self.termThr * (len(self.init_robot_pos) - 1)) / (
                    self.Notiles * len(self.init_robot_pos))
            upperThres = (self.Notiles + self.termThr) / (self.Notiles * len(self.init_robot_pos))

            success = True

            # main optimization loop
            iteration = 0

            while iteration <= self.MaxIter and not cancelled:
                self.assign()
                ConnectedMultiplierList = np.ones((len(self.init_robot_pos), self.rows, self.cols))
                ConnectedRobotRegions = np.zeros(len(self.init_robot_pos))
                plainErrors = np.zeros((len(self.init_robot_pos)))
                divFairError = np.zeros((len(self.init_robot_pos)))

                for idx, robot in enumerate(self.init_robot_pos):
                    ConnectedMultiplier = np.ones((self.rows, self.cols))
                    ConnectedRobotRegions[idx] = True
                    self.update_connectivity()
                    image = np.uint8(self.connectivity[idx, :, :])
                    num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)
                    if num_labels > 2:
                        ConnectedRobotRegions[idx] = False
                        BinaryRobot, BinaryNonRobot = self.constructBinaryImages(labels_im, robot)
                        ConnectedMultiplier = self.CalcConnectedMultiplier(
                            self.NormalizedEuclideanDistanceBinary(True, BinaryRobot),
                            self.NormalizedEuclideanDistanceBinary(False, BinaryNonRobot))
                    ConnectedMultiplierList[idx, :, :] = ConnectedMultiplier
                    plainErrors[idx] = self.ArrayOfElements[idx] / (
                            self.DesireableAssign[idx] * len(self.init_robot_pos))
                    if plainErrors[idx] < downThres:
                        divFairError[idx] = downThres - plainErrors[idx]
                    elif plainErrors[idx] > upperThres:
                        divFairError[idx] = upperThres - plainErrors[idx]

                if self.IsThisAGoalState(self.termThr, ConnectedRobotRegions):
                    print("\nFinal Assignment Matrix (" + str(iteration) + " Iterations, Tiles per Robot " + str(self.ArrayOfElements) + ")")
                    # print(self.A)
                    break

                TotalNegPerc = 0
                totalNegPlainErrors = 0
                correctionMult = np.zeros(len(self.init_robot_pos))

                for idx, robot in enumerate(self.init_robot_pos):
                    if divFairError[idx] < 0:
                        TotalNegPerc += np.absolute(divFairError[idx])
                        totalNegPlainErrors += plainErrors[idx]
                    correctionMult[idx] = 1

                # Restore Fairness among the different partitions
                for idx, robot in enumerate(self.init_robot_pos):
                    if totalNegPlainErrors != 0:
                        if divFairError[idx] < 0:
                            correctionMult[idx] = 1 + (plainErrors[idx] / totalNegPlainErrors) * (TotalNegPerc / 2)
                        else:
                            correctionMult[idx] = 1 - (plainErrors[idx] / totalNegPlainErrors) * (TotalNegPerc / 2)

                        criterionMatrix = self.calculateCriterionMatrix(
                            self.TilesImportance[idx],
                            self.MinimumImportance[idx],
                            self.MaximumImportance[idx],
                            correctionMult[idx],
                            divFairError[idx] < 0)

                    self.MetricMatrix[idx] = self.FinalUpdateOnMetricMatrix(
                        criterionMatrix,
                        self.MetricMatrix[idx],
                        ConnectedMultiplierList[idx, :, :])

                iteration += 1
                if self.visualization:
                    self.assignment_matrix_visualization.placeCells(iteration_number=iteration)
                # time.sleep(0.5)

            if iteration >= self.MaxIter:
                self.MaxIter = self.MaxIter / 2
                success = False
                self.termThr += 1

        self.getBinaryRobotRegions()
        return success

    def getBinaryRobotRegions(self):
        """
        Generate a Bool Matrix for every robot's tile area.
        :return: Manipulates BinaryRobotRegions
        """
        ind = np.where(self.A < len(self.init_robot_pos))
        temp = (self.A[ind].astype(int),) + ind
        self.BinaryRobotRegions[temp] = True

    def generateRandomMatrix(self):
        """
        Generates a matrix in map.shape with a random value for every tiles (around 1)
        :return: RandomMatrix
        """
        RandomMatrix = 2 * self.randomLevel * np.random.uniform(0, 1, size=(self.rows, self.cols)) + (
                1 - self.randomLevel)
        return RandomMatrix

    def FinalUpdateOnMetricMatrix(self, CM, currentMetricMatrix, CC):
        """
        Calculates the Final Metric Matrix with criterionMatrix, RandomMatrix, MetricMatrix, ConnectedMultiplierList
        :param CM: criterionMatrix
        :param currentMetricMatrix: current MetricMatrix of chosen robot which needs to get modified
        :param CC: ConnectedMultiplierMatrix of chosen robot
        :return: new MetricMatrix
        """
        MMnew = currentMetricMatrix * CM * self.generateRandomMatrix() * CC
        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):
        """
        Determines if the finishing criterion of the DARP algorithm is met.
        :param thresh: Sets the possible difference between the number of tiles per robot and their desired assignment
        :param connectedRobotRegions: needs array of 'is the tile area of robot x fully connected' or not
        :return: True, if criteria fits; False, if criteria aren't met
        """
        is_goalstate_met_array = np.full(len(self.init_robot_pos), False)

        for idx, r in enumerate(self.init_robot_pos):
            # the python criterion
            if np.absolute(self.DesireableAssign[idx] - self.ArrayOfElements[idx]) > thresh or not \
                    connectedRobotRegions[idx]:
                is_goalstate_met_array[idx] = False
            else:
                is_goalstate_met_array[idx] = True

        if is_goalstate_met_array.all():
            return True
        else:
            return False

    def update_connectivity(self):
        """
        Updates the self.connectivity maps after the last calculation.
        :return: Nothing
        """
        for idx, r in enumerate(self.init_robot_pos):
            self.connectivity[idx] = np.where(self.A == idx, 255, 0)

    def constructBinaryImages(self, area_tiles, robot_start_point):
        """
        Returns 2 maps in the given area_tiles.shape
        - robot_tiles_binary: where all tiles around + robot_start_point are ones, the rest is zero
        - nonrobot_tiles_binary: where tiles which aren't background and not around the robot_start_point are ones, rest is zero
        :param area_tiles: map of tiles with at least 3 different labels, 0 must always be the background
        :param robot_start_point: is needed to determine which area of connected tiles should be BinaryRobot area
        :return: robot_tiles_binary, nonrobot_tiles_binary
        """
        # area_map where all tiles with the value of the robot_start_point are 1s end the rest is 0
        robot_tiles_binary = np.where(area_tiles == area_tiles[robot_start_point], 1, 0)

        # background in area_tiles always has the value 0
        nonrobot_tiles_binary = np.where((area_tiles > 0) & (area_tiles != area_tiles[robot_start_point]), 1, 0)
        return robot_tiles_binary, nonrobot_tiles_binary

    def assign(self):
        self.BWlist = np.zeros((len(self.init_robot_pos), self.rows, self.cols))
        for idx, robot in enumerate(self.init_robot_pos):
            self.BWlist[idx, robot[0], robot[1]] = 1

        self.ArrayOfElements = np.zeros(len(self.init_robot_pos))
        for i in range(self.rows):
            for j in range(self.cols):
                # if non obstacle tile
                if self.GridEnv[i, j] == -1:
                    minV = self.MetricMatrix[0, i, j]  # finding minimal value from here on (argmin)
                    indMin = 0  # number of assigned robot of tile (i,j)
                    for idx, robot in enumerate(self.init_robot_pos):
                        if self.MetricMatrix[
                            idx, i, j] < minV:  # the actual decision making if distance of tile is lower for one robo startpoint than to another
                            minV = self.MetricMatrix[idx, i, j]
                            indMin = idx

                    self.A[i][j] = indMin
                    self.BWlist[indMin, i, j] = 1
                    self.ArrayOfElements[indMin] += 1

                # if obstacle tile
                elif self.GridEnv[i, j] == -2:
                    self.A[i, j] = len(self.init_robot_pos)

    # Construct Assignment Matrix
    def construct_Assignment_Matrix(self):
        Notiles = self.rows * self.cols
        self.effectiveSize = Notiles - len(self.init_robot_pos) - self.ob
        print("Effective number of tiles: " + str(self.effectiveSize))
        termThr = 0

        if self.effectiveSize % len(self.init_robot_pos) != 0:
            termThr = 1

        DesireableAssign = np.zeros(len(self.init_robot_pos))
        MaximunDist = np.zeros(len(self.init_robot_pos))
        MaximumImportance = np.zeros(len(self.init_robot_pos))
        MinimumImportance = np.zeros(len(self.init_robot_pos))

        for idx, robot in enumerate(self.init_robot_pos):
            DesireableAssign[idx] = self.effectiveSize * self.Rportions[idx]
            MinimumImportance[idx] = sys.float_info.max
            if DesireableAssign[idx] != int(DesireableAssign[idx]) and termThr != 1:
                termThr = 1  # threshold value of tiles which can be freely moved between assigned robot areas

        AllDistances = np.zeros((len(self.init_robot_pos), self.rows, self.cols))
        TilesImportance = np.zeros((len(self.init_robot_pos), self.rows, self.cols))

        for x in range(self.rows):
            for y in range(self.cols):
                tempSum = 0
                for idx, robot in enumerate(self.init_robot_pos):
                    AllDistances[idx, x, y] = np.linalg.norm(np.array(robot) - np.array((x, y)))  # E!
                    if AllDistances[idx, x, y] > MaximunDist[idx]:
                        MaximunDist[idx] = AllDistances[idx, x, y]
                    tempSum += AllDistances[idx, x, y]

                for idx, robot in enumerate(self.init_robot_pos):
                    if tempSum - AllDistances[idx, x, y] != 0:
                        TilesImportance[idx, x, y] = 1 / (tempSum - AllDistances[idx, x, y])
                    else:
                        TilesImportance[idx, x, y] = 1
                    # Todo FixMe!
                    if TilesImportance[idx, x, y] > MaximumImportance[idx]:
                        MaximumImportance[idx] = TilesImportance[idx, x, y]

                    if TilesImportance[idx, x, y] < MinimumImportance[idx]:
                        MinimumImportance[idx] = TilesImportance[idx, x, y]

        return AllDistances, termThr, Notiles, DesireableAssign, TilesImportance, MinimumImportance, MaximumImportance

    def calculateCriterionMatrix(self, TilesImportance, MinimumImportance, MaximumImportance, correctionMult,
                                 smallerthan0):
        """
        Generates a new correction multiplier matrix.
        If self.importance is True: TilesImportance influence is calculated.
        :param TilesImportance:
        :param MinimumImportance:
        :param MaximumImportance:
        :param correctionMult:
        :param smallerthan0:
        :return: returnCrit
        """
        if self.importance:
            if smallerthan0:
                returnCrit = (TilesImportance - MinimumImportance) * (
                        (correctionMult - 1) / (MaximumImportance - MinimumImportance)) + 1
            else:
                returnCrit = (TilesImportance - MinimumImportance) * (
                        (1 - correctionMult) / (MaximumImportance - MinimumImportance)) + correctionMult
        else:
            returnCrit = np.full(TilesImportance.shape, correctionMult)

        return returnCrit

    def CalcConnectedMultiplier(self, dist1, dist2):
        """
        Calculates the ConnectedMultiplier between the binary robot tiles (connected area) and the binary non-robot tiles
        :param dist1: Must contain the euclidean distances of all tiles around the binary robot tiles
        :param dist2: Must contain the euclidean distances of all tiles around the binary non-robot tiles
        :return: ConnectedMultiplier array in the shape of the whole area (dist1.shape & dist2.shape)
        """
        returnM = dist1 - dist2  # 2 numpy ndarray subtracted, shortform of np.subtract, returns a freshly allocated array
        MaxV = np.max(returnM)
        MinV = np.min(returnM)

        returnM = (returnM - MinV) * ((2 * self.CCvariation) / (MaxV - MinV)) + (1 - self.CCvariation)

        return returnM

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryMap):
        """
        Calculates the euclidean distances of the tiles around a given binary(non-)robot map and normalizes it.
        :param RobotR: True: given BinaryMap is area of tiles around the robot start point (BinaryRobot); False: if BinaryNonRobot tiles area and not background
        :param BinaryMap: area of tiles as binary map
        :return: Normalized distances map of the given binary (non-/)robot map in BinaryMap.shape
        """
        distRobot = ndimage.morphology.distance_transform_edt(np.logical_not(BinaryMap))
        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        # Normalization
        if RobotR:
            distRobot = ((distRobot - MinV) / (MaxV - MinV)) + 1  # why range 1 to 2 and not 0 to 1?
        else:
            distRobot = ((distRobot - MinV) / (MaxV - MinV))  # range 0 to 1

        return distRobot
