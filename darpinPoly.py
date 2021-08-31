from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
from Visualization import visualize_paths
import sys


class DARPinPoly(DARP):
    def __init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions):
        DARP.__init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions)

        if not self.success:
            print("DARP did not manage to find a solution for the given configuration!")
            sys.exit(0)

        for mode in range(4):
            print("mode", mode)
            MSTs = self.calculateMSTs(self.BinaryRobotRegions, self.droneNo, self.rows, self.cols, mode)
            AllRealPaths = []
            for r in range(self.droneNo):
                ct = CalculateTrajectories(self.rows, self.cols, MSTs[r])
                ct.initializeGraph(self.CalcRealBinaryReg(self.BinaryRobotRegions[r], self.rows, self.cols), True)
                ct.RemoveTheAppropriateEdges()
                ct.CalculatePathsSequence(4 * self.init_robot_pos[r][0] * self.cols + 2 * self.init_robot_pos[r][1])
                AllRealPaths.append(ct.PathSequence)

            TypesOfLines = np.zeros((self.rows*2, self.cols*2, 2))
            for r in range(self.droneNo):
                flag = False
                for connection in AllRealPaths[r]:
                    if flag:
                        if TypesOfLines[connection[0]][connection[1]][0] == 0:
                            indxadd1 = 0
                        else:
                            indxadd1 = 1

                        if TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                            indxadd2 = 0
                        else:
                            indxadd2 = 1
                    else:
                        if not (TypesOfLines[connection[0]][connection[1]][0] == 0):
                            indxadd1 = 0
                        else:
                            indxadd1 = 1
                        if not (TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                            indxadd2 = 0
                        else:
                            indxadd2 = 1

                    flag = True
                    if connection[0] == connection[2]:
                        if connection[1] > connection[3]:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                    else:
                        if (connection[0] > connection[2]):
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 1

            subCellsAssignment = np.zeros((2*self.rows, 2*self.cols))
            for i in range(self.rows):
                for j in range(self.cols):
                    subCellsAssignment[2 * i][2 * j] = self.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j] = self.A[i][j]
                    subCellsAssignment[2 * i][2 * j + 1] = self.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j + 1] = self.A[i][j]

            image = visualize_paths(AllRealPaths, subCellsAssignment, self.droneNo, self.color)
            # image.visualize_darp_area()
            image.visualize_paths(mode)

    def CalcRealBinaryReg(self, BinaryRobotRegion, rows, cols):
        temp = np.zeros((2*rows, 2*cols))
        RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
        for i in range(2*rows):
            for j in range(2*cols):
                temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
                if temp[i, j] == 0:
                    RealBinaryRobotRegion[i, j] = False
                else:
                    RealBinaryRobotRegion[i, j] = True

        return RealBinaryRobotRegion

    def calculateMSTs(self, BinaryRobotRegions, droneNo, rows, cols, mode):
        MSTs = []
        for r in range(self.droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(self.BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs


if __name__ == '__main__':
    nx, ny = 10, 10
    MaxIter = 80000
    CCvariation = 0.01
    randomLevel = 0.0001
    dcells = 2
    importance = False
    notEqualPortions = True
    initial_positions = [[0, 0], [4, 5], [7, 4], [8, 2]]
    portions = [0.1, 0.3, 0.4, 0.2]
    obstacles_positions = [[5, 5], [5, 6], [6, 5]]
    poly = DARPinPoly(nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions)
