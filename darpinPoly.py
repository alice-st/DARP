from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
from Visualization import visualize_paths
import sys
import argparse
from turns import turns

class DARPinPoly(DARP):
    def __init__(self, nx, ny, notEqualPortions, pos, portions, obs_pos, visualization,
                 MaxIter=80000, CCvariation=0.01, randomLevel=0.0001, dcells=2, importance=False):

        darp_instance = DARP(nx, ny, notEqualPortions, pos, portions, obs_pos, visualization,
                      MaxIter=MaxIter, CCvariation=CCvariation, randomLevel=randomLevel, dcells=dcells, importance=importance)

        success = darp_instance.findTrajectories()

        if not success:
            print("DARP did not manage to find a solution for the given configuration!")
            sys.exit(0)

        mode_to_drone_turns = dict()

        for mode in range(4):
            MSTs = self.calculateMSTs(darp_instance.BinaryRobotRegions, darp_instance.droneNo, darp_instance.rows, darp_instance.cols, mode)
            AllRealPaths = []
            for r in range(darp_instance.droneNo):
                ct = CalculateTrajectories(darp_instance.rows, darp_instance.cols, MSTs[r])
                ct.initializeGraph(self.CalcRealBinaryReg(darp_instance.BinaryRobotRegions[r], darp_instance.rows, darp_instance.cols), True)
                ct.RemoveTheAppropriateEdges()
                ct.CalculatePathsSequence(4 * darp_instance.init_robot_pos[r][0] * darp_instance.cols + 2 * darp_instance.init_robot_pos[r][1])
                AllRealPaths.append(ct.PathSequence)

            TypesOfLines = np.zeros((darp_instance.rows*2, darp_instance.cols*2, 2))
            for r in range(darp_instance.droneNo):
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

            subCellsAssignment = np.zeros((2*darp_instance.rows, 2*darp_instance.cols))
            for i in range(darp_instance.rows):
                for j in range(darp_instance.cols):
                    subCellsAssignment[2 * i][2 * j] = darp_instance.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j] = darp_instance.A[i][j]
                    subCellsAssignment[2 * i][2 * j + 1] = darp_instance.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j + 1] = darp_instance.A[i][j]

            drone_turns = turns(AllRealPaths)
            drone_turns.count_turns()
            mode_to_drone_turns[mode] = drone_turns

            if darp_instance.visualization:
                image = visualize_paths(AllRealPaths, subCellsAssignment, darp_instance.droneNo, darp_instance.color)
                image.visualize_paths(mode)

        print("\nResults:\n")
        for mode, val in mode_to_drone_turns.items():
            print(mode, val)

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
        for r in range(droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-grid',
        default=(10, 10),
        type=int,
        nargs=2,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-obs_pos',
        default=[],
        nargs='*',
        type=int,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-in_pos',
        default=[1, 3, 9],
        nargs='*',
        type=int,
        help='Initial Positions of the robots (default: (1, 3, 9))')
    argparser.add_argument(
        '-nep',
        action='store_true',
        help='Not Equal Portions shared between the Robots in the Grid (default: False)')
    argparser.add_argument(
        '-portions',
        default=[0.2, 0.3, 0.5],
        nargs='*',
        type=float,
        help='Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))')
    argparser.add_argument(
        '-vis',
        action='store_true',
        help='Visualize results (default: False)')
    args = argparser.parse_args()

    rows, cols = args.grid
    poly = DARPinPoly(rows, cols, args.nep, args.in_pos, args.portions, args.obs_pos, args.vis)
