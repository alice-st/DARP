import pickle
import unittest
from darp import DARP
import numpy as np
from multiRobotPathPlanner import MultiRobotPathPlanner
from parameterized import parameterized, parameterized_class
from nose.tools import assert_true, assert_almost_equal

# DARP parameters
MaxIter = 8000
CCvariation = 0.01
randomLevel = 0.0001
dcells = 2
importance = False

cases = [
    MultiRobotPathPlanner(10, 10,
                          False, [1, 3, 9],
                          [0.2, 0.3, 0.5], [],
                          False),
    MultiRobotPathPlanner(20, 20,
                          False, [1, 50, 76, 83],
                          [0.2, 0.3, 0.5], [3, 4, 5, 6, 7, 8, 99, 100, 101, 102],
                          False),
    MultiRobotPathPlanner(50, 50,
                          False, [1, 2, 3, 4],
                          [0.2, 0.3, 0.5], [],
                          False),
    MultiRobotPathPlanner(50, 50,
                          False, [500, 624, 1500, 2159, 2],
                          [0.2, 0.3, 0.5], [2490, 2491, 2492, 2493, 2494, 2495, 2496, 2497, 2498, 2499],
                          False)
]

@parameterized_class(('multiRobot', 'case'), [
    (cases[0], 1),
    (cases[1], 2),
    (cases[2], 3),
    (cases[3], 4)])


class TestDARP(unittest.TestCase):
    def test_DARP_assignment_matrix(self):   
        with open(f'unitTests/test{self.case}_AssignmentMatrix.pickle', 'rb') as handle:
            groundTruthAssignmentMatrix = pickle.load(handle)
        assert_true(np.allclose(self.multiRobot.darp_instance.A, groundTruthAssignmentMatrix, atol=1))
        
    def test_DARP_return_paths(self):
        with open(f'unitTests/test{self.case}_returnPaths.pickle', 'rb') as handle:
            groundTruthReturnPaths = pickle.load(handle)
        assert_almost_equal(self.multiRobot.best_case.paths, groundTruthReturnPaths)
        
    def test_DARP_iterations(self):
        with open(f'unitTests/test{self.case}_iterations.pickle', 'rb') as handle:
            groundTruthIterations = pickle.load(handle)    
        assert_true(self.multiRobot.iterations <= groundTruthIterations)

    def test_DARP_Execution_time(self):
        with open(f'unitTests/test{self.case}_execution_time.pickle', 'rb') as handle:
            groundTruthExecutionTime = pickle.load(handle)    
        assert_true(self.multiRobot.execution_time <= groundTruthExecutionTime + 1e-01)
