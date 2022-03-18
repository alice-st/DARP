import sys
import pygame
from pygame.locals import KEYDOWN, K_q
import numpy as np
import time
from sklearn.preprocessing import MinMaxScaler

# CONSTANTS:
BLACK = (0, 0, 0)
GREY = (160, 160, 160)

class visualize_paths():
    def __init__(self, AllRealPaths, subCellsAssignment, DroneNo, color):
        self.AllRealPaths = AllRealPaths
        self.subCellsAssignment = subCellsAssignment
        min_max_scaler = MinMaxScaler(feature_range=(0, 800))
        self.dimensions = min_max_scaler.fit_transform(np.array([self.subCellsAssignment.shape[0], self.subCellsAssignment.shape[1], 0]).reshape(-1, 1)).ravel()

        self.DroneNo = DroneNo
        self._VARS = {'surf': False,
                      'gridWH': (self.dimensions[0], self.dimensions[1]),
                      'gridOrigin': (0, 0),
                      'gridCellsX': self.subCellsAssignment.shape[0],
                      'gridCellsY': self.subCellsAssignment.shape[1],
                      'lineWidth': 2}
        self.color = color

    def visualize_paths(self, mode):
        pygame.init()
        self._VARS['surf'] = pygame.display.set_mode((self.dimensions[1], self.dimensions[0]))
        pygame.display.set_caption('Mode: ' + str(mode))
        while True:
            keep_going = self.checkEvents()
            if not keep_going:
                break
            self._VARS['surf'].fill(GREY)
            self.drawSquareGrid(self._VARS['gridOrigin'],
                                self._VARS['gridWH'],
                                self._VARS['gridCellsX'],
                                self._VARS['gridCellsY'])
            self.placeCells()
            pygame.display.update()

    def placeCells(self):
        cellBorder = 0
        celldimX = (self._VARS['gridWH'][0]/self._VARS['gridCellsX'])
        celldimY = (self._VARS['gridWH'][1]/self._VARS['gridCellsY'])
        
        for r in range(self.DroneNo):
            for point in self.AllRealPaths[r]:
                color = pygame.Color(255, 0, 0)
                pygame.draw.line(self._VARS['surf'],
                                 self.color[r],
                                 (self._VARS['gridOrigin'][0] + (celldimX*point[1] + celldimX/2),
                                  self._VARS['gridOrigin'][1] + (celldimY*point[0]) + celldimY/2),
                                 (self._VARS['gridOrigin'][0] + (celldimX*point[3]) + celldimX/2,
                                  self._VARS['gridOrigin'][1] + (celldimY*point[2]) + celldimY/2), width=4)

        cellBorder = 0

        for row in range(self.subCellsAssignment.shape[0]):
            for column in range(self.subCellsAssignment.shape[1]):
                if (self.subCellsAssignment[row][column] == self.DroneNo):
                    self.drawSquareCell(
                        self._VARS['gridOrigin'][0] + (celldimX*column)
                        + self._VARS['lineWidth']/2,
                        self._VARS['gridOrigin'][1] + (celldimY*row)
                        + self._VARS['lineWidth']/2,
                        celldimX, celldimY, BLACK)

    # Draw filled rectangle at coordinates
    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(
         self._VARS['surf'], color,
         (x, y, dimX, dimY)
        )

    def drawSquareGrid(self, origin, gridWH, cellsX, cellsY):
        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = (0, 0)

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y), self._VARS['lineWidth'])

        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x,
           CONTAINER_WIDTH_HEIGHT[0] + cont_y), self._VARS['lineWidth'])

        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT[0]), self._VARS['lineWidth'])
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x,
           CONTAINER_WIDTH_HEIGHT[0] + cont_y), self._VARS['lineWidth'])

        # Get cell size, just one since its a square grid.
        cellSizeX = CONTAINER_WIDTH_HEIGHT[0]/cellsX
        cellSizeY = CONTAINER_WIDTH_HEIGHT[1]/cellsY

        for x in range(cellsY):
            pygame.draw.line(
               self._VARS['surf'], BLACK,
               (cont_x + (cellSizeX * x), cont_y),
               (cont_x + (cellSizeX * x), CONTAINER_WIDTH_HEIGHT[0] + cont_y), 2)
        for y in range(cellsX):
        # # HORIZONTAl DIVISIONS
            pygame.draw.line(
              self._VARS['surf'], BLACK,
              (cont_x, cont_y + (cellSizeY*y)),
              (cont_x + CONTAINER_WIDTH_HEIGHT[1], cont_y + (cellSizeY*y)), 2)

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == KEYDOWN and event.key == K_q):
                pygame.quit()
                return False
        return True


class darp_area_visualization(object):
    def __init__(self, Assignment_matrix, DroneNo, color, init_robot_pos):
        self.Assignment_matrix = Assignment_matrix
        min_max_scaler = MinMaxScaler(feature_range=(0, 800))
        dimensions = min_max_scaler.fit_transform(np.array([self.Assignment_matrix.shape[0], self.Assignment_matrix.shape[1], 0]).reshape(-1, 1)).ravel()

        self.DroneNo = DroneNo
        self._VARS = {'surf': False,
                      'gridWH': (dimensions[0], dimensions[1]),
                      'gridOrigin': (0, 0),
                      'gridCellsX': self.Assignment_matrix.shape[0],
                      'gridCellsY': self.Assignment_matrix.shape[1],
                      'lineWidth': 2}
        self.color = color
        self.init_robot_pos_colors = [np.clip((r[0] - 20, r[1] + 20, r[2] - 20), 0, 255).tolist() for r in self.color]
        self.init_robot_pos = init_robot_pos
        pygame.init()
        self._VARS['surf'] = pygame.display.set_mode((dimensions[1], dimensions[0]))
        self.checkEvents()
        self._VARS['surf'].fill(GREY)
        self.drawSquareGrid(self._VARS['gridOrigin'], self._VARS['gridWH'], 
                            self._VARS['gridCellsX'], self._VARS['gridCellsY'])
        self.placeCells(self.Assignment_matrix)
        pygame.display.set_caption('Assignment Matrix')
        pygame.display.update()
        # time.sleep(5)

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == KEYDOWN and event.key == K_q:
                pygame.quit()
                sys.exit()

    def drawSquareGrid(self, origin, gridWH, cellsX, cellsY):
        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = (0, 0)

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y), self._VARS['lineWidth'])

        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x,
           CONTAINER_WIDTH_HEIGHT[0] + cont_y), self._VARS['lineWidth'])

        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT[0]), self._VARS['lineWidth'])
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT[1] + cont_x,
           CONTAINER_WIDTH_HEIGHT[0] + cont_y), self._VARS['lineWidth'])

        # Get cell size, just one since its a square grid.
        cellSizeX = CONTAINER_WIDTH_HEIGHT[0]/cellsX
        cellSizeY = CONTAINER_WIDTH_HEIGHT[1]/cellsY

        for x in range(cellsY):
            pygame.draw.line(
               self._VARS['surf'], BLACK,
               (cont_x + (cellSizeX * x), cont_y),
               (cont_x + (cellSizeX * x), CONTAINER_WIDTH_HEIGHT[0] + cont_y), 2)
        for y in range(cellsX):
        # # HORIZONTAl DIVISIONS
            pygame.draw.line(
              self._VARS['surf'], BLACK,
              (cont_x, cont_y + (cellSizeY*y)),
              (cont_x + CONTAINER_WIDTH_HEIGHT[1], cont_y + (cellSizeY*y)), 2)

        pygame.display.update()
    
    def placeCells(self, Assignment_matrix, iteration_number=0):
        celldimX = (self._VARS['gridWH'][0]/self._VARS['gridCellsX'])
        celldimY = (self._VARS['gridWH'][1]/self._VARS['gridCellsY'])

        for row in range(self.Assignment_matrix.shape[0]):
            for column in range(self.Assignment_matrix.shape[1]):
                if (self.Assignment_matrix[row][column] == self.DroneNo):
                    self.drawSquareCell(
                        self._VARS['gridOrigin'][0] + (celldimX*column)
                        + self._VARS['lineWidth']/2,
                        self._VARS['gridOrigin'][1] + (celldimY*row)
                        + self._VARS['lineWidth']/2,
                        celldimX, celldimY, BLACK)
                    continue
                for r in range(self.DroneNo):
                    if self.init_robot_pos[r] == (row, column):
                        self.drawSquareCell(
                            self._VARS['gridOrigin'][0] + (celldimX * column)
                            + self._VARS['lineWidth'] / 2,
                            self._VARS['gridOrigin'][1] + (celldimY * row)
                            + self._VARS['lineWidth'] / 2,
                            celldimX, celldimY, self.init_robot_pos_colors[r])
                        continue
                    else:
                        if self.Assignment_matrix[row][column] == r:
                            self.drawSquareCell(
                                self._VARS['gridOrigin'][0] + (celldimX*column)
                                + self._VARS['lineWidth']/2,
                                self._VARS['gridOrigin'][1] + (celldimY*row)
                                + self._VARS['lineWidth']/2,
                                celldimX, celldimY, self.color[r])
       
        self.drawSquareGrid(self._VARS['gridOrigin'], self._VARS['gridWH'], 
                            self._VARS['gridCellsX'], self._VARS['gridCellsY'])
        
        pygame.display.set_caption('Assignment Matrix [Iteration: ' + str(iteration_number) + ']')
        pygame.display.update()

    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(
         self._VARS['surf'], color,
         (x, y, dimX, dimY)
        )