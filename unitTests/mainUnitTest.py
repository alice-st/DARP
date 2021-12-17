import pickle
import unittest

# TODO naming
from darp import DARP
import numpy as np

from multiRobotPathPlanner import MultiRobotPathPlanner


class TestDARP(unittest.TestCase):

    def test_darp_trajectories(self):
        # DARP parameters
        self.MaxIter =80000
        self.CCvariation = 0.01
        self.randomLevel = 0.0001
        self.dcells = 2
        self.importance = False

        # Problem setup
        nx = 10
        ny = 10
        notEqualPortions = False
        initial_positions = [1, 3, 9]
        portions = [0.2, 0.3, 0.5]
        obstacles_positions = []
        visualization = False
        # Initialize DARP
        multiRobot = MultiRobotPathPlanner(nx, ny, notEqualPortions, initial_positions, portions, obstacles_positions, visualization,
                             MaxIter=self.MaxIter, CCvariation=self.CCvariation, randomLevel=self.randomLevel, dcells=self.dcells,
                             importance=self.importance)
        # Divide areas based on robots initial positions
        success = multiRobot.darp_instance.divideRegions()
        self.assertTrue(success)
        groundTruthAssigmentMatrix = np.array([[0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 1., 1., 1., 1., 2., 2., 2.],
                                                [0., 0., 0., 0., 1., 1., 2., 2., 2., 2.],
                                                [0., 0., 0., 0., 1., 1., 2., 2., 2., 2.],
                                                [0., 0., 0., 0., 0., 1., 2., 2., 2., 2.]])

        np.testing.assert_array_equal(multiRobot.darp_instance.A, groundTruthAssigmentMatrix)

        with open('test1_returnPaths.pickle', 'rb') as handle:
            groundTruthReturnPaths = pickle.load(handle)

        self.assertAlmostEqual(multiRobot.returnPaths, groundTruthReturnPaths)

