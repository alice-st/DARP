import sys
import pygame
from pygame.locals import KEYDOWN, K_q
import numpy as np
import time

# CONSTANTS:
SCREENSIZE = WIDTH, HEIGHT = 800, 800
BLACK = (0, 0, 0)
GREY = (160, 160, 160)


class visualize_paths():
    def __init__(self, AllRealPaths, subCellsAssignment, DroneNo, color):
        self.AllRealPaths = AllRealPaths
        self.subCellsAssignment = subCellsAssignment
        self.DroneNo = DroneNo
        self._VARS = {'surf': False,
                      'gridWH': 800,
                      'gridOrigin': (0, 0),
                      'gridCells': self.subCellsAssignment.shape[0],
                      'lineWidth': 2}
        self.color = color

    def visualize_paths(self, mode):
        pygame.init()
        self._VARS['surf'] = pygame.display.set_mode(SCREENSIZE)
        pygame.display.set_caption('Mode: ' + str(mode))
        while True:
            keep_going = self.checkEvents()
            if not keep_going:
                break
            self._VARS['surf'].fill(GREY)
            self.drawSquareGrid(self._VARS['gridOrigin'],
                                self._VARS['gridWH'],
                                self._VARS['gridCells'])
            self.placeCells()
            pygame.display.update()

    def visualize_darp_area(self):
        pygame.init()
        self._VARS['surf'] = pygame.display.set_mode(SCREENSIZE)
        while True:
            keep_going = self.checkEvents()
            if not keep_going:
                break
            self._VARS['surf'].fill(GREY)
            self.drawSquareGrid(self._VARS['gridOrigin'],
                                self._VARS['gridWH'],
                                self._VARS['gridCells'])
            self.darp_area()
            pygame.display.update()

    def darp_area(self):
        cellBorder = 1
        celldimX = celldimY = (self._VARS['gridWH']/self._VARS['gridCells']) - (cellBorder*2)
        for row in range(self.subCellsAssignment.shape[0]):
            for column in range(self.subCellsAssignment.shape[1]):
                if (self.subCellsAssignment[column][row] == self.DroneNo):
                    self.drawSquareCell(
                        self._VARS['gridOrigin'][0] + (celldimY*row)
                        + cellBorder + (2*row*cellBorder) + self._VARS['lineWidth']/2,
                        self._VARS['gridOrigin'][1] + (celldimX*column)
                        + cellBorder + (2*column*cellBorder) + self._VARS['lineWidth']/2,
                        celldimX, celldimY, BLACK)
                    continue
                for r in range(self.DroneNo):
                    if(self.subCellsAssignment[column][row] == r):
                        self.drawSquareCell(
                            self._VARS['gridOrigin'][0] + (celldimY*row)
                            + cellBorder + (2*row*cellBorder) + self._VARS['lineWidth']/2,
                            self._VARS['gridOrigin'][1] + (celldimX*column)
                            + cellBorder + (2*column*cellBorder) + self._VARS['lineWidth']/2,
                            celldimX, celldimY, self.color[r])

        pygame.display.update()

    def placeCells(self):
        cellBorder = 0
        celldimX = celldimY = (self._VARS['gridWH']/self._VARS['gridCells']) - (cellBorder*2)
        for r in range(self.DroneNo):
            for point in self.AllRealPaths[r]:
                # size = (300, 300)
                color = pygame.Color(255, 0, 0)
                pygame.draw.line(self._VARS['surf'],
                                 self.color[r],
                                 (self._VARS['gridOrigin'][0] + (celldimX*point[1] + celldimX/2),
                                  self._VARS['gridOrigin'][1] + (celldimY*point[0]) + celldimY/2),
                                 (self._VARS['gridOrigin'][0] + (celldimX*point[3]) + celldimX/2,
                                  self._VARS['gridOrigin'][1] + (celldimY*point[2]) + celldimY/2), width=4)

        cellBorder = 1
        celldimX = celldimY = (self._VARS['gridWH']/self._VARS['gridCells']) - (cellBorder*2)
        for row in range(self.subCellsAssignment.shape[0]):
            for column in range(self.subCellsAssignment.shape[1]):
                if (self.subCellsAssignment[column][row] == self.DroneNo):
                    self.drawSquareCell(
                        self._VARS['gridOrigin'][0] + (celldimY*row)
                        + 1 + (2*row*1) + self._VARS['lineWidth']/2,
                        self._VARS['gridOrigin'][1] + (celldimX*column)
                        + 1 + (2*column*1) + self._VARS['lineWidth']/2,
                        celldimX, celldimY, BLACK)

        pygame.display.update()

    # Draw filled rectangle at coordinates
    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(
         self._VARS['surf'], color,
         (x, y, dimX, dimY)
        )

    def drawSquareGrid(self, origin, gridWH, cells):

        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = origin

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y), self._VARS['lineWidth'])
        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, CONTAINER_WIDTH_HEIGHT + cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x,
           CONTAINER_WIDTH_HEIGHT + cont_y), self._VARS['lineWidth'])
        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT), self._VARS['lineWidth'])
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x,
           CONTAINER_WIDTH_HEIGHT + cont_y), self._VARS['lineWidth'])

        # Get cell size, just one since its a square grid.
        cellSize = CONTAINER_WIDTH_HEIGHT/cells

        for x in range(cells):
            pygame.draw.line(
               self._VARS['surf'], BLACK,
               (cont_x + (cellSize * x), cont_y),
               (cont_x + (cellSize * x), CONTAINER_WIDTH_HEIGHT + cont_y), 2)
        # # HORIZONTAl DIVISIONS
            pygame.draw.line(
              self._VARS['surf'], BLACK,
              (cont_x, cont_y + (cellSize*x)),
              (cont_x + CONTAINER_WIDTH_HEIGHT, cont_y + (cellSize*x)), 2)

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == KEYDOWN and event.key == K_q):
                pygame.quit()
                return False
        return True


class darp_area_visualization(object):
    def __init__(self, Assignment_matrix, DroneNo, color):
        self.Assignment_matrix = Assignment_matrix
        self.DroneNo = DroneNo
        self._VARS = {'surf': False,
                      'gridWH': 800,
                      'gridOrigin': (0, 0),
                      'gridCells': self.Assignment_matrix.shape[0],
                      'lineWidth': 2}
        self.color = color
        pygame.init()
        self._VARS['surf'] = pygame.display.set_mode(SCREENSIZE)
        self.checkEvents()
        self._VARS['surf'].fill(GREY)
        self.drawSquareGrid(
         self._VARS['gridOrigin'], self._VARS['gridWH'], self._VARS['gridCells'])
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

    def drawSquareGrid(self, origin, gridWH, cells):

        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = origin

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y), self._VARS['lineWidth'])
        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, CONTAINER_WIDTH_HEIGHT + cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x,
           CONTAINER_WIDTH_HEIGHT + cont_y), self._VARS['lineWidth'])
        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (cont_x, cont_y),
          (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT), self._VARS['lineWidth'])
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
          self._VARS['surf'], BLACK,
          (CONTAINER_WIDTH_HEIGHT + cont_x, cont_y),
          (CONTAINER_WIDTH_HEIGHT + cont_x,
           CONTAINER_WIDTH_HEIGHT + cont_y), self._VARS['lineWidth'])

        # Get cell size, just one since its a square grid.
        cellSize = CONTAINER_WIDTH_HEIGHT/cells

        for x in range(cells):
            pygame.draw.line(
               self._VARS['surf'], BLACK,
               (cont_x + (cellSize * x), cont_y),
               (cont_x + (cellSize * x), CONTAINER_WIDTH_HEIGHT + cont_y), 2)
        # # HORIZONTAl DIVISIONS
            pygame.draw.line(
              self._VARS['surf'], BLACK,
              (cont_x, cont_y + (cellSize*x)),
              (cont_x + CONTAINER_WIDTH_HEIGHT, cont_y + (cellSize*x)), 2)

    def placeCells(self, Assignment_matrix):
        cellBorder = 1
        celldimX = celldimY = (self._VARS['gridWH']/self._VARS['gridCells']) - (cellBorder*2)
        for row in range(self.Assignment_matrix.shape[0]):
            for column in range(self.Assignment_matrix.shape[1]):
                if (self.Assignment_matrix[column][row] == self.DroneNo):
                    self.drawSquareCell(
                        self._VARS['gridOrigin'][0] + (celldimY*row)
                        + cellBorder + (2*row*cellBorder) + self._VARS['lineWidth']/2,
                        self._VARS['gridOrigin'][1] + (celldimX*column)
                        + cellBorder + (2*column*cellBorder) + self._VARS['lineWidth']/2,
                        celldimX, celldimY, BLACK)
                    continue
                for r in range(self.DroneNo):
                    if(self.Assignment_matrix[column][row] == r):
                        self.drawSquareCell(
                            self._VARS['gridOrigin'][0] + (celldimY*row)
                            + cellBorder + (2*row*cellBorder) + self._VARS['lineWidth']/2,
                            self._VARS['gridOrigin'][1] + (celldimX*column)
                            + cellBorder + (2*column*cellBorder) + self._VARS['lineWidth']/2,
                            celldimX, celldimY, self.color[r])

        pygame.display.update()

    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(
         self._VARS['surf'], color,
         (x, y, dimX, dimY)
        )