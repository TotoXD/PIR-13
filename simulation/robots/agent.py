# Classe agent permettant de définir le comportement des robots

from cmath import inf, sqrt
from enum import unique
from operator import index
from urllib.robotparser import RobotFileParser
from mesa import Agent
import math
import random
from . import config
import time
from pandas import array
from robots.MapRobot import *
from robots.utils import *

# Classe mère Robot

class Robot(Agent):

    shared_map = GlobalMap(MAX_ROBOTS) # Map en variable partagée entre tous les robots


    def __init__(self, unique_id, radius, model, view_angle=90):

        super().__init__(unique_id, model)

        # Définitions des variables

        self.frontier_dest_set = False
        self.frontier_dest = None
        self.trajectory = []
        self.radius = radius
        self.capDeg = 0
        self.angle_rotation_total = 0
        self.newCapDeg = 0
        self.view_angle = view_angle
        self.local_map = LocalMap(self.pos)
        self.next_move = None
        self.next_move_set = False
        self.rotation_angle = 45

    # Fonctions utiles au robot pour faire des actions 

    def isExplorationOver(self):
        # if there is no more frontier available -> exploration is completed
        return len(Robot.shared_map.getFrontiersToExplore()) == 0

    def getCellsCoordinatesBorderRadius(self, cell):
        # récupération de toutes les cellules potentiielement visibles à 360

        start = time.time() # Start computation time calc

        potentially_visible_cells = self.getVisibleCells(360, cell)

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return potentially_visible_cells

    def isRotationUseful(self):
        set_cells = self.getCellsCoordinatesBorderRadius(self.pos)
        start = time.time() # Start computation time calc
        for cell in set_cells:
            # si la cellule n'a pas été explorée et qu'elle n'est pas non plus définie comme un mur
            if (cell not in Robot.shared_map.getCellsExplored()) and (cell not in Robot.shared_map.getWalls()):
                stop = time.time() # End computation time calc
                config.computation_time += stop-start # Find computation time
                return True

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return False

    def getRadius(self):
        return self.radius

    def step(self): # Appelée par model.py pour faire avancer le robot

        start = time.time() # Start computation time calc

        # moving if we can
        self.move()
        # exploring our environment and updating the set model.explored_cells
        self.robotVision()

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time # Find computation time

    def getVisibleCells(self, angle, cell): # Récupère les cellules dans le champ de vision du robot

        start = time.time() # Start computation time calc

        x, y = cell
        visible_neighbors = set()
        visible_neighbors.add(cell)

        angles = self.getViewAngle(angle)
        for deg in angles:
            rad = math.radians(deg)

            for c in range(self.radius + 1):
                wallFound = False
                cX = round(math.sin(rad) * c + x)
                cY = round(math.cos(rad) * c + y)

                if not self.model.grid.out_of_bounds((cX, cY)):
                    visible_neighbors.add((cX, cY))
                    for agents in self.model.grid.get_cell_list_contents([(cX, cY)]):
                        if type(agents) is Wall:
                            wallFound = True
                if wallFound:
                    break

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return visible_neighbors

    def getVisibleCellsProbability(self, angle, cell):

        start = time.time() # Start computation time calc

        x, y = cell
        visible_neighbors = set()
        visible_neighbors.add(cell)

        angles = self.getViewAngle(angle)
        for deg in angles:
            rad = math.radians(deg)

            for c in range(self.radius):

                wallFound = False
                cX = round(math.sin(rad) * c + x)
                cY = round(math.cos(rad) * c + y)
                config.computation_cost += 1 # Add 1 to total computation cost

                if not self.model.grid.out_of_bounds((cX, cY)):
                    visible_neighbors.add((cX, cY))
                    # print("cell = " + str(cX) + ", " + str(cY) + "\n")
                    for agents in self.model.grid.get_cell_list_contents([(cX, cY)]):
                        if type(agents) is Wall:
                            wallFound = True
                if wallFound:

                    stop = time.time() # End computation time calc
                    config.computation_time += stop-start # Find computation time

                    break

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return visible_neighbors

    def robotVision(self):

        start = time.time() # Start computation time calc

        visible_neighbors = self.getVisibleCells(self.view_angle, self.pos)

        # visible_neighbors are the cells visible by the robots
        for cell in visible_neighbors:
            self.model.cells_visible.add(cell)

            contains_wall = False
            for cell_content in self.model.grid.get_cell_list_contents([cell]):
                if type(cell_content) is Wall:
                    contains_wall = True

                    stop = time.time() # End computation time calc
                    config.computation_time += stop-start # Find computation time

                    break

            # si la cellule est un mur on la considère comme inatteignable
            if contains_wall:
                self.local_map.addWall(cell)
                Robot.shared_map.addWall(cell)
            else:
                # sinon on l'ajoute aux cellules explorées
                self.local_map.addCellExplored(cell)
                Robot.shared_map.addCellExplored(cell, self.unique_id)

        # determine the frontiers
        Robot.shared_map.determineFrontiers()

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

    def getViewAngle(self, view_angle):
        start = time.time() # Start computation time calc
        newAngles = []

        step=1

        if view_angle == 360:
            step=35
            
        for i in range(0, view_angle,step):
            config.computation_cost += 1 # Add 1 to total computation cost
            newAngles.append((self.capDeg - view_angle // 2 + i) % 360)

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return newAngles

    def doFullRotation(self):

        start = time.time() # Start computation time calc

        self.rotate(self.rotation_angle)
        self.angle_rotation_total += self.rotation_angle

        if self.angle_rotation_total >= 360:
            self.angle_rotation_total = 0
            self.rotation_destination_reached = False

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

    def rotate(self, angleDiff):
        self.capDeg = (self.capDeg + angleDiff) % 360
        config.rotation_cost += 1
        config.rotation_cost_angle = (
            (2*math.pi*(self.rotation_angle))/360)*config.rotation_cost

    def calcNewCap(self, cell):
        xC, yC = cell
        xR, yR = self.pos
        diffX, diffY = xR - xC, yR - yC

        if diffX < 0:
            # La case suivante est sur la droite
            if diffY < 0:
                # La case suivante est au dessus à droite
                return 45
            elif diffY == 0:
                # La case suivante est directement à droite
                return 90
            else:
                # La case suivante en bas à droite
                return 135
        elif diffX == 0:
            # La case suivante se situe au dessus ou en dessous
            if diffY < 0:
                # La case suivante est au dessus
                return 0
            else:
                # La case suivante en dessous
                return 180
        else:
            # La case suivante est à gauche
            if diffY < 0:
                # La case suivante est au dessus à gauche
                return 315
            elif diffY == 0:
                # La case suivante est directement à gauche
                return 270
            else:
                # La case suivante en bas à gauche
                return 225

    def updateLocalMapCoordinates(self):
        self.local_map.updatePosition(self.pos)

    def removeFromVisible(self, x0, x1, y0, y1, neigh):
        start = time.time() # Start computation time calc
        for xCoord in range(x0, x1):
            for yCoord in range(y0, y1):
                if (xCoord, yCoord) in neigh:
                    neigh.remove((xCoord, yCoord))

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return neigh

    def getAccessibleNeighborhood(self, position):
        # get the neighborhood cells
        # moore is to include the diagonals in the neighborhood or not

        start = time.time() # Start computation time calc

        list_neighborhood = self.local_map.getAccessibleNeighbors(
            position, Robot.shared_map.getListPositionsRobots())

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return list_neighborhood

    def updatePositionOnMaps(self):
        start = time.time() # Start computation time calc

        self.updateLocalMapCoordinates()
        Robot.shared_map.updatePositionRobot(self.unique_id, self.pos)

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

    """
    Methods for Yamauchi/Burgard
    """  

    def findTrajectory(self, destination):
        return Robot.shared_map.WPA(self.pos, Robot.shared_map.getListPositionsRobots(), self.model.diagonal, destination,
                                    self.unique_id)

    def findOptimalTrajectory(self, destination):
        return Robot.shared_map.WPA_without_robots_interferences(self.pos, Robot.shared_map.getListPositionsRobots(), self.model.diagonal,
                                                                 destination)

    def getNextMove(self):
        # voir si il y a des déplacements possibles

        start = time.time() # Start computation time calc

        self.next_move = self.pos
        if len(self.trajectory) != 0:
            self.next_move = self.trajectory.pop()

        self.newCapDeg = self.calcNewCap(self.next_move)

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return self.next_move

    def checkOrientation(self):

        start = time.time() # Start computation time calc

        well_oriented = True

        # si on n'est pas orienté dans le bon sens
        if self.capDeg != self.newCapDeg:
            if abs(self.capDeg - self.newCapDeg) % 330 > self.rotation_angle:

                # On cherche la direction dans laquelle tourner
                if not ((0 <= self.capDeg - self.newCapDeg <= 180) or (
                        -180 >= self.capDeg - self.newCapDeg >= -360)):
                    # sens horaire
                    self.rotate(self.rotation_angle)
                else:
                    # sens anti-horaire
                    self.rotate(-self.rotation_angle)
            else:
                self.capDeg = self.newCapDeg
            well_oriented = False

        stop = time.time() # End computation time calc
        config.computation_time += stop-start # Find computation time

        return well_oriented

    def isNextMoveOccupied(self):
        # Check whether next_move cell is occupied by a robot
        return self.next_move in Robot.shared_map.getListPositionsRobots()


class RobotRandom(Robot):
    """ Class implementing a random strategy """

    def __init__(self, unique_id, radius, model, view_angle=90, rotation_angle=45, proba_move=66, proba_turn_right=50):
        super().__init__(unique_id, radius, model, view_angle)
        self.proba_move = proba_move
        self.proba_turn_right = proba_turn_right
        self.rotation_angle = rotation_angle

    def move(self):
        # STRATEGY IMPLEMENTED : random moves with only Neuman neighbors
        possible_steps = self.getAccessibleNeighborhood(self.pos)
        config.computation_cost += 1 # Add 1 to total computation cost

        # Si le robot avait déjà prévu une destination avant, mais qu'il devait encore tourner pour l'atteindre
        if self.next_move_set:
            # si le robot est déjà bien orienté
            if self.checkOrientation():
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost
        else:
            # Le robot a proba_move d'avancer
            if(random.randint(1, 100) > self.proba_move):
                # Si le robot tourne, une chance sur 2 de tourner à gauche ou à droite
                if(random.randint(1, 100) > self.proba_turn_right):
                    self.rotate(-self.rotation_angle)
                    config.computation_cost += 1 # Add 1 to total computation cost

                else:
                    self.rotate(self.rotation_angle)
                    config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # if there are possible moves
                if len(possible_steps) > 0:

                    self.next_move = self.random.choice(possible_steps)
                    self.newCapDeg = self.calcNewCap(self.next_move)
                    config.computation_cost += 1 # Add 1 to total computation cost
                    # si le robot est déjà bien orienté
                    if self.checkOrientation():
                        self.model.grid.move_agent(self, self.next_move)
                        self.next_move_set = False

                        # sinon on a tourné ce tour-ci
                        self.updatePositionOnMaps()
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        self.next_move_set = True
                        config.computation_cost += 1 # Add 1 to total computation cost


class RobotRandomBehaviors(Robot):
    """ Class implementing a random strategy with behaviors"""

    def __init__(self, unique_id, radius, model, view_angle=90, rotation_angle=45, proba_move_routine=85, proba_move_standard=50, proba_move_anxious=15):
        super().__init__(unique_id, radius, model, view_angle)
        if(unique_id % 3 == 0):
            self.proba_move = proba_move_routine
            self.proba_turn_right = 50
        elif(unique_id % 3 == 1):
            self.proba_move = proba_move_standard
            self.proba_turn_right = 50
        elif(unique_id % 3 == 2):
            self.proba_move = proba_move_anxious
            self.proba_turn_right = 50
        self.next_move_set = False
        self.rotation_angle = rotation_angle

    def move(self):
        # STRATEGY IMPLEMENTED : random moves with only Neuman neighbors
        possible_steps = self.getAccessibleNeighborhood(self.pos)
        config.computation_cost += 1 # Add 1 to total computation cost
        # Si le robot avait déjà prévu une destination avant, mais qu'il devait encore tourner pour l'atteindre
        if self.next_move_set:
            # si le robot est déjà bien orienté
            if self.checkOrientation():

                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost
        else:
            # Le robot a proba_move d'avancer
            if(random.randint(1, 100) > self.proba_move):
                # Si le robot tourne, une chance sur 2 de tourner à gauche ou à droite
                if(random.randint(1, 100) > self.proba_turn_right):
                    self.rotate(-self.rotation_angle)
                    config.computation_cost += 1 # Add 1 to total computation cost
                else:
                    self.rotate(self.rotation_angle)
                    config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # if there are possible moves
                if len(possible_steps) > 0:
                    self.next_move = self.random.choice(possible_steps)
                    self.newCapDeg = self.calcNewCap(self.next_move)
                    config.computation_cost += 1 # Add 1 to total computation cost
                    # si le robot est déjà bien orienté
                    if self.checkOrientation():

                        self.model.grid.move_agent(self, self.next_move)
                        self.next_move_set = False

                        # sinon on a tourné ce tour-ci
                        self.updatePositionOnMaps()
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        self.next_move_set = True
                        config.computation_cost += 1 # Add 1 to total computation cost


class RobotRandomAnt(Robot):
    """ Class implementing a random ant based strategy """

    def __init__(self, unique_id, radius, model, view_angle=90, rotation_angle=45, min_times_moving_forward=2, max_times_moving_forward=8):
        super().__init__(unique_id, radius, model, view_angle)
        self.next_move_set = False
        self.rotation_angle = rotation_angle
        self.proba_move = 75
        self.proba_turn_right = 50
        self.times_moving_forward = 0
        self.min_times_moving_forward = min_times_moving_forward
        self.max_times_moving_forward = max_times_moving_forward

    def move(self):
        # STRATEGY IMPLEMENTED : random moves with only Neuman neighbors
        possible_steps = self.getAccessibleNeighborhood(self.pos)
        config.computation_cost += 1 # Add 1 to total computation cost

        # Si le robot avait déjà prévu une destination avant, mais qu'il devait encore tourner pour l'atteindre, ou qu'il doit encore avancer
        if self.next_move_set:
            # si le robot est déjà bien orienté
            if self.checkOrientation():

                self.model.grid.move_agent(self, self.next_move)
                self.times_moving_forward -= 1
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost
        else:
            if self.times_moving_forward > 0:
                (x, y) = self.next_move
                if self.newCapDeg == 0:
                    (x, y) = (x, y+1)
                elif self.newCapDeg == 45:
                    (x, y) = (x+1, y+1)
                elif self.newCapDeg == 90:
                    (x, y) = (x+1, y)
                elif self.newCapDeg == 135:
                    (x, y) = (x+1, y-1)
                elif self.newCapDeg == 180:
                    (x, y) = (x, y-1)
                elif self.newCapDeg == 225:
                    (x, y) = (x-1, y-1)
                elif self.newCapDeg == 270:
                    (x, y) = (x-1, y)
                elif self.newCapDeg == 315:
                    (x, y) = (x-1, y+1)

                if (x, y) in possible_steps:
                    self.next_move = (x, y)
                    self.newCapDeg = self.calcNewCap(self.next_move)

                    # si le robot est déjà bien orienté
                    if self.checkOrientation():

                        self.model.grid.move_agent(self, self.next_move)
                        self.times_moving_forward -= 1
                        self.next_move_set = False

                        # sinon on a tourné ce tour-ci
                        self.updatePositionOnMaps()
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        self.next_move_set = True
                        config.computation_cost += 1 # Add 1 to total computation cost
                else:
                    self.times_moving_forward = 0

            else:
                # Le robot a proba_move d'avancer
                if(random.randint(1, 100) > self.proba_move):
                    # Si le robot tourne, une chance sur 2 de tourner à gauche ou à droite
                    if(random.randint(1, 100) > self.proba_turn_right):
                        self.rotate(-self.rotation_angle)
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        self.rotate(self.rotation_angle)
                        config.computation_cost += 1 # Add 1 to total computation cost
                else:
                    # if there are possible moves
                    if len(possible_steps) > 0:
                        self.next_move = self.random.choice(possible_steps)
                        self.newCapDeg = self.calcNewCap(self.next_move)
                        config.computation_cost += 1 # Add 1 to total computation cost

                        self.times_moving_forward = random.randint(
                            self.min_times_moving_forward, self.max_times_moving_forward)

                        # si le robot est déjà bien orienté
                        if self.checkOrientation():

                            self.model.grid.move_agent(self, self.next_move)
                            self.times_moving_forward -= 1
                            self.next_move_set = False

                            # sinon on a tourné ce tour-ci
                            self.updatePositionOnMaps()
                            config.computation_cost += 1 # Add 1 to total computation cost
                        else:
                            self.next_move_set = True
                            config.computation_cost += 1 # Add 1 to total computation cost


class RobotYamauchi(Robot):
    """ Class implementing Yamauchi strategy """

    def __init__(self, unique_id, radius, model, view_angle=90):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0

    def move(self):
        # si la frontière n'est en fait plus une frontière
        if self.frontier_dest_set:

            if self.frontier_dest == self.pos:
                # seulement dans ce cas tu fais une rotation pelo
                self.rotation_destination_reached = True
                self.frontier_dest_set = False
                self.next_move_set = False
                config.computation_cost += 1 # Add 1 to total computation cost

            elif self.frontier_dest not in Robot.shared_map.getFrontiersToExplore():
                self.frontier_dest_set = False
                self.next_move_set = False
                config.computation_cost += 1 # Add 1 to total computation cost

        # je fais une rotation pour explorer mon environnement
        # changer cela pour qu'il y ait un calcul de la part du robot
        # qu'il consulte sa map globale avant et regarde si c'est vraiment utile de faire une rotation
        if self.rotation_destination_reached:
            # if it's still useful to do a rotation
            if self.isRotationUseful():
                self.doFullRotation()
                config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # on décide d'arrêter la rotation
                self.rotation_destination_reached = False

        else:
            # si le robot n'a pas encore de destination à atteindre
            if not self.frontier_dest_set:
                # recherche de la frontière la plus proche
                self.shared_map.clusterDBSCAN(radius=7)

                self.frontier_dest, self.trajectory = Robot.shared_map.findClosestFrontier(
                    self.pos, self.unique_id)
                self.frontier_dest_set = True
                config.computation_cost += 1 # Add 1 to total computation cost

            # si on a pas encore définit le prochain move
            if not self.next_move_set:
                self.next_move = self.getNextMove()
                config.computation_cost += 1 # Add 1 to total computation cost

                # si la case est occupée par un robot
                if self.isNextMoveOccupied():
                    # on recalcule la trajectoire
                    start = time.time() # Start computation time calc
                    self.trajectory = self.findTrajectory(self.frontier_dest)
                    stop = time.time() # End computation time calc
                    config.computation_time += stop-start # Find computation time

                    # on récupère le next move
                    self.next_move = self.getNextMove()
                    config.computation_cost += 1 # Add 1 to total computation cost
                self.next_move_set = True
                config.computation_cost += 1 # Add 1 to total computation cost

            # si le robot est déjà bien orienté
            if self.checkOrientation():

                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost


class RobotBurgard(Robot):

    # Class implementing Burgard strategy

    def __init__(self, unique_id, radius, model, view_angle, beta, bRadius,):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0
        self.destination_reached = False
        self.beta = beta
        self.bRadius = bRadius

    def move(self):

        self.destination_reached = False

        # If the destination is not a frontier anymore
        if self.frontier_dest_set and self.frontier_dest not in Robot.shared_map.getFrontiersToExplore():
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the destination is the actual position
        if self.frontier_dest_set and self.frontier_dest == self.pos:
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            self.rotation_destination_reached = True  # Want the 360 rotation
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the rotation is asked
        if self.rotation_destination_reached:
            # if it's still useful to do a rotation
            if self.isRotationUseful():
                self.doFullRotation()
                config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # we decide to stop the rotation
                self.rotation_destination_reached = False
                config.computation_cost += 1 # Add 1 to total computation cost

        else:
            # If the destination is not reached
            if not self.frontier_dest_set:
                # Search the destination
                self.frontier_dest = self.destinationAssignment()
                self.frontier_dest_set = True

                # Compute the trajectory
                start = time.time() # Start computation time calc
                self.trajectory = self.findOptimalTrajectory(
                    self.frontier_dest)
                stop = time.time() # End computation time calc
                config.computation_time += stop-start # Find computation time
                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot has not next move defined
            if not self.next_move_set:
                self.next_move = self.getNextMove()
                self.next_move_set = True
                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot's orientation is fine
            if self.checkOrientation():
                # If the next move cell is occupied by another robot
                if self.isNextMoveOccupied():
                    # If a robot is on its destination and next to it
                    if self.frontier_dest in Robot.shared_map.getListPositionsRobots():
                        self.next_move = self.pos
                        self.frontier_dest = None
                        self.frontier_dest_set = False
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        # set another trajectory to the same destination
                        start = time.time() # Start computation time calc
                        self.trajectory = self.findTrajectory(
                            self.frontier_dest)
                        stop = time.time() # End computation time calc
                        config.computation_time += stop-start # Find computation time
                        # set the next move
                        self.next_move = self.getNextMove()
                        config.computation_cost += 1 # Add 1 to total computation cost

                # move the robot the next move cell
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False
                # update the map
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost
            if self.pos == self.rotation_destination_reached:
                self.destination_reached = True
                config.computation_cost += 1 # Add 1 to total computation cost

    def cost(self):
        """ Cost computation """
        position = self.pos
        config.computation_cost += 1 # Add 1 to total computation cost
        numberOfFrontier = len(
            Robot.shared_map.getFrontiersToExplore())  # should be local but we consider communication at anytime
        # Initialise the cost to 0 for the actual cell
        self.local_map.addCost(self.pos, 0)
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_next = set()  # Cells who will be computed
        # Cells computed in the previous iteration
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_prev = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost
        # Cells already computed in all iterations
        cells_already_calculated = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost

        # while there is still a frontier to explore
        while numberOfFrontier != 0 and len(cells_prev) > 0:
            # Catch the cell's neighbours
            for pos in cells_prev:
                # print(pos)
                config.computation_cost += 1 # Add 1 to total computation cost
                cells_next = cells_next.union(
                    set(self.model.grid.get_neighborhood(pos, moore=True, include_center=False)))

            # Remove the cells already computed
            cells_next -= cells_already_calculated
            config.computation_cost += 1 # Add 1 to total computation cost

            # Initialise the cost to infinity
            for cell in cells_next:
                self.local_map.addCost(cell, math.inf)
                config.computation_cost += 1 # Add 1 to total computation cost

            # Update the cost of the cells in cells_next
            for elem in cells_next:
                neighbours_list = self.model.grid.get_neighborhood(
                    elem, moore=True, include_center=False)
                # Take the neighbours of that cell
                for neighbour in neighbours_list:
                    # If this neighbour has no cost, give it
                    if neighbour not in self.local_map.getEveryCost():
                        self.local_map.addCost(neighbour, math.inf)
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # If neighbour a wall, its occupancy = 1
                    if neighbour in Robot.shared_map.getWalls():
                        occupancy_value = 1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        occupancy_value = 0.1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # Cost comparison, the minimum is the best
                    if self.local_map.getCellCost(elem) > self.local_map.getCellCost(neighbour) + get_distance(
                            neighbour, elem) * occupancy_value:
                        config.computation_cost += 1 # Add 1 to total computation cost
                        self.local_map.addCost(elem, self.local_map.getCellCost(neighbour) + get_distance(neighbour,
                                                                                                          elem) * occupancy_value)
                # If the cell is a frontier, we reduce its number
                if elem in self.local_map.getFrontiersToExplore():
                    if self.local_map.getCellCost(elem) < math.inf:
                        numberOfFrontier -= 1
                        config.computation_cost += 1 # Add 1 to total computation cost

            cells_prev = cells_next
            cells_already_calculated = cells_already_calculated.union(
                cells_prev)
            config.computation_cost += 1 # Add 1 to total computation cost

    def destinationAssignment(self):

        self.cost()  # compute the cost

        destination = None
        destination_interest = -math.inf
        utilityCostRatio = self.getAttribute("beta")  # β
        # The quantity β ≥ 0 determines the relative importance of
        # utility versus cost

        config.computation_cost += 1 # Add 1 to total computation cost

        # Update the utility to all the frontier cells
        for frontierCell, utility in Robot.shared_map.getEveryFrontierUtility().items():

            # If there is a better destination
            if destination_interest < utility - utilityCostRatio * self.local_map.getCellCost(frontierCell):
                destination_interest = utility - utilityCostRatio * \
                    self.local_map.getCellCost(frontierCell)
                destination = frontierCell
                config.computation_cost += 1 # Add 1 to total computation cost

        if destination is None:
            config.computation_cost += 1 # Add 1 to total computation cost
            return self.pos
        self.reduceUtility(destination)
        config.computation_cost += 1 # Add 1 to total computation cost

        return destination

    # Reduce utility of frontiers near the destination
    def reduceUtility(self, destination):

        # For the potential cells visible when the destination reached
        for hypotheticalFrontierCell in self.getFrontiersInRadius(destination):

            # Reduce the utility for the other robots
            if hypotheticalFrontierCell in Robot.shared_map.getEveryFrontierUtility():
                if get_distance(destination, hypotheticalFrontierCell) <= (self.getAttribute("bRadius")):

                    config.computation_cost += 1 # Add 1 to total computation cost

                    Robot.shared_map.addUtility(hypotheticalFrontierCell, (get_distance(
                        destination, hypotheticalFrontierCell) / self.getAttribute("bRadius")))

 # Pour récupérer les paramètres de Burgard pour les calculs

    def getAttribute(self, attribute):
        if attribute == "beta":
            return self.beta
        if attribute == "bRadius":
            return self.bRadius

    def getFrontiersInRadius(self, cell):
        # récupération de toutes les cellules potentiielement visibles à 360
        potentially_visible_cells = set()

        # Get frontiers
        for frontier in Robot.shared_map.getFrontiersToExplore():
            if get_distance(cell, frontier) <= (self.getAttribute("bRadius")):
                if(self.local_map.getCellCost(frontier) <= (self.getAttribute("bRadius"))):
                    potentially_visible_cells.add(frontier)

        return potentially_visible_cells


class RobotEntropy(Robot):

    # Class implementing BurgardEntropy strategy

    def __init__(self, unique_id, radius, model, view_angle, beta,bRadius,):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0
        self.destination_reached = False
        self.beta = beta
        self.bRadius = bRadius

    def move(self):

        self.destination_reached = False

        # If the destination is not a frontier anymore
        if self.frontier_dest_set and self.frontier_dest not in Robot.shared_map.getFrontiersToExplore():
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the destination is the actual position
        if self.frontier_dest_set and self.frontier_dest == self.pos:
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            self.rotation_destination_reached = True  # Want the 360 rotation
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the rotation is asked
        if self.rotation_destination_reached:
            # if it's still useful to do a rotation
            if self.isRotationUseful():
                self.doFullRotation()
                config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # we decide to stop the rotation
                self.rotation_destination_reached = False
                config.computation_cost += 1 # Add 1 to total computation cost

        else:
            # If the destination is not reached
            if not self.frontier_dest_set:
                # Search the destination
                self.frontier_dest = self.destinationAssignment()
                self.frontier_dest_set = True

                # Compute the trajectory
                self.trajectory = self.findOptimalTrajectory(
                    self.frontier_dest)
                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot has not next move defined
            if not self.next_move_set:
                self.next_move = self.getNextMove()
                self.next_move_set = True
                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot's orientation is fine
            if self.checkOrientation():
                # If the next move cell is occupied by another robot
                if self.isNextMoveOccupied():
                    # If a robot is on its destination and next to it
                    if self.frontier_dest in Robot.shared_map.getListPositionsRobots():
                        self.next_move = self.pos
                        self.frontier_dest = None
                        self.frontier_dest_set = False
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        # set another trajectory to the same destination
                        self.trajectory = self.findTrajectory(
                            self.frontier_dest)
                        # set the next move
                        self.next_move = self.getNextMove()
                        config.computation_cost += 1 # Add 1 to total computation cost

                # move the robot the next move cell
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False
                # update the map
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost
            if self.pos == self.rotation_destination_reached:
                self.destination_reached = True
                config.computation_cost += 1 # Add 1 to total computation cost

    def cost(self):
        """ Cost computation """
        position = self.pos
        config.computation_cost += 1 # Add 1 to total computation cost
        numberOfFrontier = len(
            Robot.shared_map.getFrontiersToExplore())  # should be local but we consider communication at anytime
        # Initialise the cost to 0 for the actual cell
        self.local_map.addCost(self.pos, 0)
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_next = set()  # Cells who will be computed
        # Cells computed in the previous iteration
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_prev = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost
        # Cells already computed in all iterations
        cells_already_calculated = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost

        # while there is still a frontier to explore
        while numberOfFrontier != 0 and len(cells_prev) > 0:
            # Catch the cell's neighbours
            for pos in cells_prev:
                # print(pos)
                config.computation_cost += 1 # Add 1 to total computation cost
                cells_next = cells_next.union(
                    set(self.model.grid.get_neighborhood(pos, moore=True, include_center=False)))

            # Remove the cells already computed
            cells_next -= cells_already_calculated
            config.computation_cost += 1 # Add 1 to total computation cost

            # Initialise the cost to infinity
            for cell in cells_next:
                self.local_map.addCost(cell, math.inf)
                config.computation_cost += 1 # Add 1 to total computation cost

            # Update the cost of the cells in cells_next
            for elem in cells_next:
                neighbours_list = self.model.grid.get_neighborhood(
                    elem, moore=True, include_center=False)
                # Take the neighbours of that cell
                for neighbour in neighbours_list:
                    # If this neighbour has no cost, give it
                    if neighbour not in self.local_map.getEveryCost():
                        self.local_map.addCost(neighbour, math.inf)
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # If neighbour a wall, its occupancy = 1
                    if neighbour in Robot.shared_map.getWalls():
                        occupancy_value = 1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        occupancy_value = 0.1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # Cost comparison, the minimum is the best
                    if self.local_map.getCellCost(elem) > self.local_map.getCellCost(neighbour) + get_distance(
                            neighbour, elem) * occupancy_value:
                        config.computation_cost += 1 # Add 1 to total computation cost
                        self.local_map.addCost(elem, self.local_map.getCellCost(neighbour) + get_distance(neighbour,
                                                                                                          elem) * occupancy_value)
                # If the cell is a frontier, we reduce its number
                if elem in self.local_map.getFrontiersToExplore():
                    if self.local_map.getCellCost(elem) < math.inf:
                        numberOfFrontier -= 1
                        config.computation_cost += 1 # Add 1 to total computation cost

            cells_prev = cells_next
            cells_already_calculated = cells_already_calculated.union(
                cells_prev)
            config.computation_cost += 1 # Add 1 to total computation cost

    def destinationAssignment(self):

        self.cost()  # compute the cost

        destination = None
        destination_interest = -math.inf
        utilityCostRatio = self.getAttribute("beta")  # β
        # The quantity β ≥ 0 determines the relative importance of
        # utility versus cost

        config.computation_cost += 1 # Add 1 to total computation cost

        # Update the utility to all the frontier cells
        for frontierCell, utility in Robot.shared_map.getEveryFrontierUtility().items():

            if destination_interest < utility - utilityCostRatio * self.local_map.getCellCost(frontierCell):
                destination_interest = utility - utilityCostRatio * self.local_map.getCellCost(frontierCell)
                destination = frontierCell
                config.computation_cost += 1 # Add 1 to total computation cost

        if destination is None:
            config.computation_cost += 1 # Add 1 to total computation cost
            return self.pos

        self.reduceUtility(destination)
        config.computation_cost += 1 # Add 1 to total computation cost

        return destination

    # Occupancy probability update
    def determineOccupancy(self):
        # If cell is empty, p=0.9
        for cell in Robot.shared_map.getAlreadySeen():
            Robot.shared_map.addOccupancy(cell, 0.9)
        # If cell is a wall, p=0.1
        for cell in Robot.shared_map.getWalls():
            Robot.shared_map.addOccupancy(cell, 0.1)

    def determineEntropy(self):

    # Entropy update of all the cells

        # For every cell
        for cell in Robot.shared_map.getAlreadySeen():
            config.computation_cost += 1 # Add 1 to total computation cost
            Robot.shared_map.addEntropy(cell, (-1*Robot.shared_map.getCellOccupancy(cell)*math.log(Robot.shared_map.getCellOccupancy(
                cell), 2) - ((1-Robot.shared_map.getCellOccupancy(cell))*math.log((1-Robot.shared_map.getCellOccupancy(cell)), 2))))

        # For every frontier
        for cell in Robot.shared_map.getFrontiersToExplore():
            
            # For every cell in the frontier FOV
            for cellInRadius in self.getVisibleCells(360, cell):

                config.computation_cost += 1 # Add 1 to total computation cost

                # Add occupancy of p=0.5 to undiscovered cells and calc entropy
                if cellInRadius not in Robot.shared_map.getAlreadySeen() and cellInRadius not in Robot.shared_map.getFrontiersToExplore():
                    Robot.shared_map.addOccupancy(cellInRadius, 0.5)
                    Robot.shared_map.addEntropy(cellInRadius, -1*Robot.shared_map.getCellOccupancy(cellInRadius)*math.log(Robot.shared_map.getCellOccupancy(cellInRadius), 2)
                                                - ((1-Robot.shared_map.getCellOccupancy(cellInRadius))*math.log(1-Robot.shared_map.getCellOccupancy(cellInRadius), 2)))


    def determineEntropyFrontiers(self):

        # Compute the entropy of frontiers

        # For every frontier
        for cell in Robot.shared_map.getFrontiersToExplore():

            # For every cell in the frontier FOV
            for cellInRadius in self.getVisibleCells(360, cell):

                config.computation_cost += 1 # Add 1 to total computation cost

                # If cell is not a frontier
                if (get_distance(cell, cellInRadius) < (self.radius)) and cellInRadius not in Robot.shared_map.getFrontiersToExplore():

                    # Add its entropy to the frontier
                    Robot.shared_map.addEntropy(cell, Robot.shared_map.getCellEntropy(cell)+Robot.shared_map.getCellEntropy(cellInRadius))

    def reduceUtility(self, destination):

        # Reduce utility around the destination

        # Get visible cells from the destination
        neighbours_destination=self.getVisibleCells(360, destination)

        # Entropy of destination = 0
        Robot.shared_map.addEntropy(destination,0)

        # For the potential cells visible when the destination reached
        for hypotheticalFrontierCell in neighbours_destination:

            # If the visible cell is a frontier
            if hypotheticalFrontierCell in Robot.shared_map.getFrontiersToExplore()  and (get_distance(destination, hypotheticalFrontierCell) < (self.radius)):
                # For every visible cell from this frontier
                for cell in self.getVisibleCells(360, hypotheticalFrontierCell):

                    # If it's not a frontier and also exists in the destination's FOV
                    if cell not in Robot.shared_map.getFrontiersToExplore():
                        if Robot.shared_map.getCellEntropy(cell) and (cell in neighbours_destination) and ( hypotheticalFrontierCell != destination):
                            # The information is shared between them
                            Robot.shared_map.addEntropy(hypotheticalFrontierCell, (Robot.shared_map.getCellEntropy(
                                                hypotheticalFrontierCell)-Robot.shared_map.getCellEntropy(cell)))

                config.computation_cost += 1 # Add 1 to total computation cost

    # Pour récupérer les paramètres de Burgard pour les calculs

    def getAttribute(self, attribute):
        if attribute == "beta":
            return self.beta
        if attribute == "bRadius":
            return self.bRadius


def get_distance(cell1, cell2):
    """Get the distance between two point.
    Args:
        pos_1, pos_2: Coordinate tuples for both points.
    """
    x1, y1 = cell1
    x2, y2 = cell2
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return math.sqrt(dx * dx + dy * dy)


class SearcherDrosophila(Robot):
    # implements a target-searching strategy

    sharedBeliefGrid = BeliefGrid()

    def __init__(self, unique_id, radius, model, view_angle=90):
        super().__init__(unique_id, radius, model, view_angle)
        # initial probability that the target is indeed present
        self.presenceProbability = 0.75
        self.beliefPresenceThreshold = 0.95
        self.beliefAbsenceThreshold = 0.005
        # probabilities of false positive and false negative
        # self.pFalseNegative = 0.1
        # self.pFalsePositive = 0.1
        self.destination = None
        self.destination_set = False

    def move(self):
        # Check if the destination was not explored this turn
        if self.destination in Robot.shared_map.getWalls() or self.pos == self.destination:
            self.destination_set = False
            self.trajectory = []
            self.next_move = None
            config.computation_cost += 1 # Add 1 to total computation cost

        if not self.destination_set:
            self.destination = self.assignDestination()
            if self.destination is not None:
                self.destination_set = True

                start = time.time() # Start computation time calc
                self.trajectory = self.findTrajectory(self.destination)
                stop = time.time() # End computation time calc
                config.computation_time += stop-start # Find computation time
                config.computation_cost += 1 # Add 1 to total computation cost

                if self.trajectory == -1:
                    self.destination_set = False
                    # print("Cannot reach dest")
                    self.trajectory = []
                    SearcherDrosophila.sharedBeliefGrid.unreachableCells.add(
                        self.destination)
                    config.computation_cost += 1 # Add 1 to total computation cost

        self.moveOnGrid()

        # Update Belief Grid
        for visible_cell in self.getVisibleCellsProbability(self.view_angle, self.pos):
            config.computation_cost += 1 # Add 1 to total computation cost
            SearcherDrosophila.sharedBeliefGrid.update(
                visible_cell, self.getObservation(visible_cell))
        self.updatePositionOnMaps()

    def moveOnGrid(self):
        if self.pos == self.next_move or self.next_move is None:
            self.getNextMove()
            config.computation_cost += 1 # Add 1 to total computation cost

        if self.checkOrientation():
            if self.isNextMoveOccupied():
                self.destination_set = False
                self.next_move = None
                self.trajectory = []
                config.computation_cost += 1 # Add 1 to total computation cost
            else:
                self.model.grid.move_agent(self, self.next_move)
                config.computation_cost += 1 # Add 1 to total computation cost

    def assignDestination(self):
        # Find the cell with the highest probability of presence in the grid and assign it as destination
        dest = None
        maxCellBelief = -1

        # We change the way the grid is browsed each time to avoid blocking situations
        if random.random() < 0.5:
            xRange = range(self.model.grid.width)
            config.computation_cost += 1 # Add 1 to total computation cost
        else:
            xRange = range(self.model.grid.width - 1, -1, -1)
            config.computation_cost += 1 # Add 1 to total computation cost
        if random.random() < 0.5:
            yRange = range(self.model.grid.height)
            config.computation_cost += 1 # Add 1 to total computation cost
        else:
            yRange = range(self.model.grid.height - 1, -1, -1)
            config.computation_cost += 1 # Add 1 to total computation cost

        # Browsing the belief Grid
        for xC in xRange:
            for yC in yRange:
                if SearcherDrosophila.sharedBeliefGrid.grid[xC][yC] > maxCellBelief and (xC, yC) not in Robot.shared_map.getWalls() and (xC, yC) not in SearcherDrosophila.sharedBeliefGrid.unreachableCells:
                    # print("newly assignedDest = " + str(xC) + ", " + str(yC) + " || and belief = " + str(SearcherDrosophila.sharedBeliefGrid.grid[xC][yC]))
                    dest = (xC, yC)
                    maxCellBelief = SearcherDrosophila.sharedBeliefGrid.grid[xC][yC]
                    config.computation_cost += 1 # Add 1 to total computation cost
        return dest

    def getObservation(self, cell):
        for agents in self.model.grid.get_cell_list_contents(cell):
            if type(agents) is Target:
                return 1
        return 0

    def getAggregateBelief(self):
        aggregateBelief = 0
        maxCellBelief = 0
        for i in range(SearcherDrosophila.sharedBeliefGrid.width):
            for j in range(SearcherDrosophila.sharedBeliefGrid.height):
                aggregateBelief += SearcherDrosophila.sharedBeliefGrid.grid[i][j]
                config.computation_cost += 1 # Add 1 to total computation cost
                if SearcherDrosophila.sharedBeliefGrid.grid[i][j] > maxCellBelief:
                    maxCellBelief = SearcherDrosophila.sharedBeliefGrid.grid[i][j]
                    config.computation_cost += 1 # Add 1 to total computation cost
        aggregateBelief = aggregateBelief / (
            SearcherDrosophila.sharedBeliefGrid.width * SearcherDrosophila.sharedBeliefGrid.height)
        config.computation_cost += 1 # Add 1 to total computation cost
        config.belief = aggregateBelief

        return aggregateBelief, maxCellBelief

    def getTargetFound(self):
        maxCellBelief = 0
        nbTargetFound = 0
        for i in range(SearcherDrosophila.sharedBeliefGrid.width):
            for j in range(SearcherDrosophila.sharedBeliefGrid.height):
                if SearcherDrosophila.sharedBeliefGrid.grid[i][j] ==0:
                    # if (i,j) not in SearcherDrosophila.sharedBeliefGrid.targetFound:
                    #     SearcherDrosophila.sharedBeliefGrid.targetFound.append((i,j))
                        nbTargetFound = nbTargetFound+1
        return nbTargetFound == SearcherDrosophila.sharedBeliefGrid.nbTarget

    # Override

    def findTrajectory(self, destination):
        config.computation_cost += 1 # Add 1 to total computation cost
        return Robot.shared_map.WPAUnexplored(self.pos, self.model.diagonal, destination)

    # Override
    def isNextMoveOccupied(self):
        # Check whether next_move cell is occupied by a robot
        # print("is next move occupied = " + str(self.next_move in Robot.shared_map.getListPositionsRobots() or self.next_move in Robot.shared_map.getWalls()))
        if self.next_move in Robot.shared_map.getListPositionsRobots() or self.next_move in Robot.shared_map.getWalls():
            config.computation_cost += 1 # Add 1 to total computation cost
            return True
        for move in self.trajectory:
            if move in Robot.shared_map.getWalls():
                config.computation_cost += 1 # Add 1 to total computation cost
                return True
        return False

    # Override
    def isExplorationOver(self):
        # If the individual cell belief of one cell goes over the threshold of presence, then we consider that a target has been found
        # If the mean cell belief is lesser than the threshold of absence, then we consider that no target is present in the map
        meanCellBelief, maxCellBelief = self.getAggregateBelief()

        # if meanCellBelief < self.beliefAbsenceThreshold or maxCellBelief > self.beliefPresenceThreshold:
        if meanCellBelief < self.beliefAbsenceThreshold:
            config.computation_cost += 1 # Add 1 to total computation cost
            return True
        # Pour finir la recherche quand l'exploration est finie
        # return len(Robot.shared_map.getFrontiersToExplore()) == 0

        # Pour finir la recherche quand toutes les cibles ont été trouvée
        return len(Robot.shared_map.getFrontiersToExplore()) == 0 or self.getTargetFound()


class SearcherDrosophilaShortest(SearcherDrosophila):
    def __init__(self, unique_id, radius, model, view_angle=90):
        super().__init__(unique_id, radius, model, view_angle)
        self.destinations_available = []

    # Override
    def move(self):

        # Check if the destination was not explored this turn
        if self.destination in Robot.shared_map.getWalls() or self.pos == self.destination:
            self.destination_set = False
            self.trajectory = []
            self.next_move = None
            config.computation_cost += 1 # Add 1 to total computation cost

        if not self.destination_set:
            self.destinations_available = self.findAvailableDestinations()
            config.computation_cost += 1 # Add 1 to total computation cost
            if self.destinations_available is not []:
                self.destination_set = True
                self.destination, self.trajectory = self.findTrajectory(
                    self.destinations_available)
                config.computation_cost += 1 # Add 1 to total computation cost
                if self.trajectory == -1:
                    self.destination_set = False
                    self.trajectory = []
                    SearcherDrosophila.sharedBeliefGrid.unreachableCells.add(
                        self.findAvailableDestinations().pop())
                    config.computation_cost += 1 # Add 1 to total computation cost

        self.moveOnGrid()
        config.computation_cost += 1 # Add 1 to total computation cost

        # Update Belief Grid
        for visible_cell in self.getVisibleCellsProbability(self.view_angle, self.pos):
            SearcherDrosophila.sharedBeliefGrid.update(
                visible_cell, self.getObservation(visible_cell))
            config.computation_cost += 1 # Add 1 to total computation cost
        self.updatePositionOnMaps()

    # Override
    def findAvailableDestinations(self):
        # Find the cell with the highest probability of presence in the grid and assign it as destination
        dest = []
        maxCellBelief = -1

        # Browsing the belief Grid
        for xC in range(self.model.grid.width):
            for yC in range(self.model.grid.height):
                if SearcherDrosophila.sharedBeliefGrid.grid[xC][yC] >= maxCellBelief and (xC, yC) not in Robot.shared_map.getWalls() and (xC, yC) not in SearcherDrosophila.sharedBeliefGrid.unreachableCells:
                    if SearcherDrosophila.sharedBeliefGrid.grid[xC][yC] == maxCellBelief:
                        # print("newly assignedDest = " + str(xC) + ", " + str(yC) + " || and belief = " + str(SearcherDrosophila.sharedBeliefGrid.grid[xC][yC]))
                        dest.append((xC, yC))
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        maxCellBelief = SearcherDrosophila.sharedBeliefGrid.grid[xC][yC]
                        dest = [(xC, yC)]
                        config.computation_cost += 1 # Add 1 to total computation cost
        return dest

    # Override
    def findTrajectory(self, destination):

        config.computation_cost += 1 # Add 1 to total computation cost
        return Robot.shared_map.findShortestTrajectory(self.pos, self.destinations_available)


class RobotMinPos(Robot):
    """ Class implementing the MinPos strategy """

    def __init__(self, unique_id, radius, model, view_angle=90):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0
        # Définies sur None, car on ne connait pas le nombre de robots, ni de frontières,
        # et qu'il est impossible de créer un tableau 2D de taille indéfinie
        self.cost_matrix = None
        self.rank_matrix = None
        self.map_has_changed = False
        self.frontiers_known = []
        self.number_of_robots = 1
        self.number_of_frontiers = 0

    def move(self):
        self.shared_map.determineFrontiers()
        frontiers = self.shared_map.getFrontiersToExplore()

        # Désactivé car prend trop de temps de calcul
        """
        # Initialisation des frontières connues
        if self.frontiers_known == [] :
            self.frontiers_known = frontiers

        # Si les frontières anciennement connues sont différentes de celles actuelles, la map a changé
        if self.frontiers_known != frontiers:
            self.map_has_changed = True
        """
        
        # Si la map a changé ou que la frontière à atteindre n'est pas définie, on va attribuer la frontière à explorer
        if self.map_has_changed or not self.frontier_dest_set:
            # On repasse à False car on va réassigner la frontière par la suite
            self.map_has_changed = False

            # print("CALCUL")
            # Calcul des rangs

            # Debug
            # print("-----------")

            robots_position = Robot.shared_map.getListPositionsRobots()

            # Debug
            """
            print("Frontières triées : ", sorted(
                frontiers, key=lambda tup: (tup[0], tup[1])))
            """

            # Debug
            """
            print("pos ", self.pos)
            """

            self.number_of_robots = len(robots_position)
            self.number_of_frontiers = len(frontiers)

            self.cost_matrix = [[0 for x in range(
                self.number_of_robots)] for y in range(self.number_of_frontiers)]

            # Debug
            """
            print(self.cost_matrix)
            print(self.number_of_robots)
            print(self.number_of_frontiers)
            """

            # Calcul de la matrice des coûts
            for robot in range(self.number_of_robots):
                frontier_index = 0
                for frontier in frontiers:
                    cost = len(self.shared_map.WPA(
                        robots_position[robot], robots_position, self.model.diagonal, frontier, robot))
                    # print(cost)
                    if cost == 0:
                        # Inatteignable
                        self.cost_matrix[frontier_index][robot] = math.inf
                    else:
                        self.cost_matrix[frontier_index][robot] = cost
                    frontier_index += 1

            # Debug
            """
            print("Tableau des cost matrix robot juste après calcul", self.unique_id)
            for r in self.cost_matrix:
                for c in r:
                    print(c, end=" ")
                print()
            """

            # .copy() doesn't work
            self.rank_matrix = [[0 for x in range(
                self.number_of_robots)] for y in range(self.number_of_frontiers)]

            # Calcul de la matrice des rangs
            current_index_matrix = 0
            for frontier in self.cost_matrix:

                sorted_frontiers = sorted(set(frontier))

                index = 0
                current_rank = 1
                for current_frontier in sorted_frontiers:
                    if current_frontier != math.inf:
                        self.rank_matrix[current_index_matrix][frontier.index(
                            current_frontier)] = current_rank
                        current_rank += 1
                        index += 1
                    # Si la frontière est inatteignable : rank de 0
                    else:
                        self.rank_matrix[current_index_matrix][frontier.index(
                            current_frontier)] = 0
                current_index_matrix += 1

            # Debug
            """
            print("Tableau des rank matrix robot", self.unique_id)
            for r in self.rank_matrix:
                for c in r:
                    print(c, end=" ")
                print()
            """

            # Attribution d'une frontière au robot :
            # On choisi tous les robots, on cherche ses rangs les plus faibles
            min_rank = math.inf
            index = 0
            robot_list_index_min = []
            list_index_min = []
            for i in range(self.number_of_robots):
                for frontier_rank in self.rank_matrix:
                    frontier_rank = frontier_rank[i]
                    if frontier_rank != 0:
                        if frontier_rank < min_rank:
                            list_index_min.clear()
                            min_rank = frontier_rank
                            list_index_min.append(index)
                        elif frontier_rank == min_rank:
                            list_index_min.append(index)

                    index += 1
                robot_list_index_min.append((list_index_min, min_rank, i))
                min_rank = math.inf
                index = 0
                list_index_min = []

            # ---------- 3 Méthodes d'attribution des frontières -----------

            # Méthode 1
            # Random sur la frontiere à choisir
            # Peu efficace
            """
            number_frontiers_to_explore = len(list_index_min)
            if number_frontiers_to_explore > 0:
                frontier_to_explore = random.randint(
                    0, number_frontiers_to_explore-1)
                self.frontier_dest_set = True
                frontiers_list = list(frontiers)
                self.frontier_dest = frontiers_list[list_index_min[frontier_to_explore]]
            """

            # Méthode 2 : MinPos2
            # Attribution de la frontière à explorer parmi celles de rang plus faible, en commençant par le robot ayant le rang de frontière min le plus élevé
            # Très efficace

            # Méthode 3 (Actuel) : MinPos
            # Méthode d'allocation des frontières de MinPos classique
            # Très efficace
            # Trie les robots par ordre de rang croissant : premier a rang le plus faible , dernier le plus élevé
            sorted_robot_list_index_min = sorted(
                robot_list_index_min, key=lambda x: x[1], reverse=False)

            # Méthode d'allocation des frontières pour Méthode 2 & 3
            frontiers_already_attributed = []
            for robot_processing in range(len(sorted_robot_list_index_min)):
                number_frontiers_to_explore = len(
                    (sorted_robot_list_index_min[robot_processing])[0])

                if number_frontiers_to_explore > 0:
                    index_min = -1
                    for i in range(len((sorted_robot_list_index_min[robot_processing])[0])):

                        if index_min == -1:
                            # Vérifie que la frontière à traiter n'est pas déjà attribuée à un autre robot
                            if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[i])] not in frontiers_already_attributed:
                                min = self.cost_matrix[((sorted_robot_list_index_min[robot_processing])[0])[
                                    i]][(sorted_robot_list_index_min[robot_processing])[2]]
                                index_min = i

                        else:
                            # Vérifie que la frontière à traiter n'est pas déjà attribuée à un autre robot
                            if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[i])] not in frontiers_already_attributed:
                                potential_min = self.cost_matrix[((sorted_robot_list_index_min[robot_processing])[0])[i]][(
                                    sorted_robot_list_index_min[robot_processing])[2]]
                                if min > potential_min:
                                    index_min = i
                                    min = potential_min

                    # Si c'est notre robot, on va lui attribuer la frontière
                    if self.unique_id == (sorted_robot_list_index_min[robot_processing])[2]:
                        # Vérifie que la frontière à attribuer n'est pas déjà attribuée à un autre robot
                        if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[index_min])] not in frontiers_already_attributed:
                            if index_min != -1:
                                self.frontier_dest = (list(frontiers))[
                                    ((sorted_robot_list_index_min[robot_processing])[0])[index_min]]
                                self.frontier_dest_set = True
                                self.trajectory = self.shared_map.WPA(
                                    self.pos, robots_position, self.model.diagonal, self.frontier_dest, self.unique_id)

                    frontiers_already_attributed.append((list(frontiers))[(
                        (sorted_robot_list_index_min[robot_processing])[0])[index_min]])
                    index_min = -1

                    # Debug
                    """             
                    print("Robot ", (sorted_robot_list_index_min[robot_processing])[2], " min : ",
                          min, " parmi couts ", self.cost_matrix, " des frontieres ", frontiers)
                    """

            # Debug
            """
            print("nb ", number_frontiers_to_explore)
            print("frontieres", len(frontiers))
            """

            # Debug
            """
            print("\nFrontières: ", frontiers)
            print("\nPosition: ", self.pos)
            print("\nObjectif: ", self.frontier_dest)
            print("\nMatrice couts: ", self.cost_matrix)
            """

        # Sinon la map n'a pas changée / la frontière est déjà attribuée
        # print("MOUVEMENT")
        if self.frontier_dest not in self.shared_map.getCellsDiscoverer():
            # si on a pas encore définit le prochain move
            if not self.next_move_set:
                self.next_move = self.getNextMove()

                # si la case est occupée par un robot
                if self.isNextMoveOccupied():
                    # on recalcule la trajectoire
                    self.trajectory = self.findTrajectory(
                        self.frontier_dest)

                    # on récupère le next move
                    self.next_move = self.getNextMove()

                self.next_move_set = True

            # si le robot est déjà bien orienté
            if self.checkOrientation():
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                self.frontier_dest_set = False
                self.rotation_destination_reached = True
        else:
            self.frontier_dest = None
            self.frontier_dest_set = False
            self.next_move = None
            self.next_move_set = None


class RobotMinPos2(Robot):
    """ Class implementing the MinPos strategy with a different frontier allocation """

    def __init__(self, unique_id, radius, model, view_angle=90):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0
        # Définies sur None, car on ne connait pas le nombre de robots, ni de frontières,
        # et qu'il est impossible de créer un tableau 2D de taille indéfinie
        self.cost_matrix = None
        self.rank_matrix = None
        self.map_has_changed = False
        self.frontiers_known = []
        self.number_of_robots = 1
        self.number_of_frontiers = 0

    def move(self):
        self.shared_map.determineFrontiers()
        frontiers = self.shared_map.getFrontiersToExplore()

        # Désactivé car prend trop de temps de calcul
        """
        # Initialisation des frontières connues
        if self.frontiers_known == [] :
            self.frontiers_known = frontiers

        # Si les frontières anciennement connues sont différentes de celles actuelles, la map a changé
        if self.frontiers_known != frontiers:
            self.map_has_changed = True
        """

        # Si la map a changé ou que la frontière à atteindre n'est pas définie, on va attribuer la frontière à explorer
        if self.map_has_changed or not self.frontier_dest_set:
            # On repasse à False car on va réassigner la frontière par la suite
            self.map_has_changed = False

            # print("CALCUL")
            # Calcul des rangs

            # Debug
            # print("-----------")

            robots_position = Robot.shared_map.getListPositionsRobots()

            # Debug
            """
            print("Frontières triées : ", sorted(
                frontiers, key=lambda tup: (tup[0], tup[1])))
            """

            # Debug
            """
            print("pos ", self.pos)
            """

            self.number_of_robots = len(robots_position)
            self.number_of_frontiers = len(frontiers)

            self.cost_matrix = [[0 for x in range(
                self.number_of_robots)] for y in range(self.number_of_frontiers)]

            # Debug
            """
            print(self.cost_matrix)
            print(self.number_of_robots)
            print(self.number_of_frontiers)
            """

            # Calcul de la matrice des coûts
            for robot in range(self.number_of_robots):
                frontier_index = 0
                for frontier in frontiers:
                    cost = len(self.shared_map.WPA(
                        robots_position[robot], robots_position, self.model.diagonal, frontier, robot))
                    # print(cost)
                    if cost == 0:
                        # Inatteignable
                        self.cost_matrix[frontier_index][robot] = math.inf
                    else:
                        self.cost_matrix[frontier_index][robot] = cost
                    frontier_index += 1

            # Debug
            """
            print("Tableau des cost matrix robot juste après calcul", self.unique_id)
            for r in self.cost_matrix:
                for c in r:
                    print(c, end=" ")
                print()
            """

            # .copy() doesn't work
            self.rank_matrix = [[0 for x in range(
                self.number_of_robots)] for y in range(self.number_of_frontiers)]

            # Calcul de la matrice des rangs
            current_index_matrix = 0
            for frontier in self.cost_matrix:

                sorted_frontiers = sorted(set(frontier))

                index = 0
                current_rank = 1
                for current_frontier in sorted_frontiers:
                    if current_frontier != math.inf:
                        self.rank_matrix[current_index_matrix][frontier.index(
                            current_frontier)] = current_rank
                        current_rank += 1
                        index += 1
                    # Si la frontière est inatteignable : rank de 0
                    else:
                        self.rank_matrix[current_index_matrix][frontier.index(
                            current_frontier)] = 0
                current_index_matrix += 1

            # Debug
            """
            print("Tableau des rank matrix robot", self.unique_id)
            for r in self.rank_matrix:
                for c in r:
                    print(c, end=" ")
                print()
            """

            # Attribution d'une frontière au robot :
            # On choisi tous les robots, on cherche ses rangs les plus faibles
            min_rank = math.inf
            index = 0
            robot_list_index_min = []
            list_index_min = []
            for i in range(self.number_of_robots):
                for frontier_rank in self.rank_matrix:
                    frontier_rank = frontier_rank[i]
                    if frontier_rank != 0:
                        if frontier_rank < min_rank:
                            list_index_min.clear()
                            min_rank = frontier_rank
                            list_index_min.append(index)
                        elif frontier_rank == min_rank:
                            list_index_min.append(index)

                    index += 1
                robot_list_index_min.append((list_index_min, min_rank, i))
                min_rank = math.inf
                index = 0
                list_index_min = []

            # Trie les robots par ordre de rang décroissant : premier a rang le plus élevé , dernier le plus faible
            sorted_robot_list_index_min = sorted(
                robot_list_index_min, key=lambda x: x[1], reverse=True)

            # ---------- 3 Méthodes d'attribution des frontières -----------

            # Méthode 1
            # Random sur la frontiere à choisir
            # Peu efficace
            """
            number_frontiers_to_explore = len(list_index_min)
            if number_frontiers_to_explore > 0:
                frontier_to_explore = random.randint(
                    0, number_frontiers_to_explore-1)
                self.frontier_dest_set = True
                frontiers_list = list(frontiers)
                self.frontier_dest = frontiers_list[list_index_min[frontier_to_explore]]
            """

            # Méthode 2 (Actuel) : MinPos2
            # Attribution de la frontière à explorer parmi celles de rang plus faible, en commençant par le robot ayant le rang de frontière min le plus élevé
            # Très efficace
            # Trie les robots par ordre de rang décroissant : premier a rang le plus élevé , dernier le plus faible
            sorted_robot_list_index_min = sorted(
                robot_list_index_min, key=lambda x: x[1], reverse=True)

            # Méthode 3 : MinPos
            # Méthode d'allocation des frontières de MinPos classique
            # Très efficace

            # Méthode d'allocation des frontières pour Méthode 2 & 3
            frontiers_already_attributed = []
            for robot_processing in range(len(sorted_robot_list_index_min)):
                number_frontiers_to_explore = len(
                    (sorted_robot_list_index_min[robot_processing])[0])

                if number_frontiers_to_explore > 0:
                    index_min = -1
                    for i in range(len((sorted_robot_list_index_min[robot_processing])[0])):

                        if index_min == -1:
                            # Vérifie que la frontière à traiter n'est pas déjà attribuée à un autre robot

                            if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[i])] not in frontiers_already_attributed:
                                min = self.cost_matrix[((sorted_robot_list_index_min[robot_processing])[0])[
                                    i]][(sorted_robot_list_index_min[robot_processing])[2]]
                                index_min = i

                        else:
                            # Vérifie que la frontière à traiter n'est pas déjà attribuée à un autre robot
                            if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[i])] not in frontiers_already_attributed:
                                potential_min = self.cost_matrix[((sorted_robot_list_index_min[robot_processing])[0])[i]][(
                                    sorted_robot_list_index_min[robot_processing])[2]]
                                if min > potential_min:
                                    index_min = i
                                    min = potential_min

                    # Si c'est notre robot, on va lui attribuer la frontière
                    if self.unique_id == (sorted_robot_list_index_min[robot_processing])[2]:
                        # Vérifie que la frontière à attribuer n'est pas déjà attribuée à un autre robot
                        if (list(frontiers))[(((sorted_robot_list_index_min[robot_processing])[0])[index_min])] not in frontiers_already_attributed:
                            if index_min != -1:
                                self.frontier_dest = (list(frontiers))[
                                    ((sorted_robot_list_index_min[robot_processing])[0])[index_min]]
                                self.frontier_dest_set = True
                                self.trajectory = self.shared_map.WPA(
                                    self.pos, robots_position, self.model.diagonal, self.frontier_dest, self.unique_id)

                    frontiers_already_attributed.append((list(frontiers))[(
                        (sorted_robot_list_index_min[robot_processing])[0])[index_min]])
                    index_min = -1

                    # Debug
                    """             
                    print("Robot ", (sorted_robot_list_index_min[robot_processing])[2], " min : ",
                          min, " parmi couts ", self.cost_matrix, " des frontieres ", frontiers)
                    """

            # Debug
            """
            print("nb ", number_frontiers_to_explore)
            print("frontieres", len(frontiers))
            """

            # Debug
            """
            print("\nFrontières: ", frontiers)
            print("\nPosition: ", self.pos)
            print("\nObjectif: ", self.frontier_dest)
            print("\nMatrice couts: ", self.cost_matrix)
            """

        # Sinon la map n'a pas changée / la frontière est déjà attribuée
        # print("MOUVEMENT")
        if self.frontier_dest not in self.shared_map.getCellsDiscoverer():
            # si on a pas encore définit le prochain move
            if not self.next_move_set:
                self.next_move = self.getNextMove()

                # si la case est occupée par un robot
                if self.isNextMoveOccupied():
                    # on recalcule la trajectoire
                    self.trajectory = self.findTrajectory(
                        self.frontier_dest)

                    # on récupère le next move
                    self.next_move = self.getNextMove()

                self.next_move_set = True

            # si le robot est déjà bien orienté
            if self.checkOrientation():
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False

                # sinon on a tourné ce tour-ci
                self.updatePositionOnMaps()
                self.frontier_dest_set = False
                self.rotation_destination_reached = True
        else:
            self.frontier_dest = None
            self.frontier_dest_set = False
            self.next_move = None
            self.next_move_set = None


class Wall(Agent):
    """
        Just a wall
        """

    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def getColor(self):
        content = self.model.grid.get_cell_list_contents(self.pos)
        for agent in content:
            if type(agent) is Cell:
                if agent.isExplored() or (self.pos in Robot.shared_map.getWalls()):
                    return "#000000"
        return "#00000000"


class Target(Agent):
    """
        A target cell that needs to be discovered
        For now it has no step method, it is considered immobile
        """

    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Cell(Agent):
    """
        A Cell than can either be explored or unexplored
        """

    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def isVisible(self):
        return self.pos in self.model.cells_visible

    def isExplored(self):
        return self.pos in Robot.shared_map.getCellsExplored()

    def isTarget(self):
        return self.model.robot_type == "Target"

    def isFakeTarget(self):
        return self.model.robot_type == "FakeTarget"

    def getColor(self):
        if self.model.robot_type not in ["SearcherDrosophila", "SearcherDrosophilaShortest", "BurgardDrosophile"]:
            if self.isFrontier():
                # if (RobotEntropy.shared_map.getCellEntropy(self)==0):
                #     return "#FFFFFF"
                # else:
                return "#0df910"  # if not self.isVisible() else "#689F38"
            elif self.isVisible():
                return "#D84315"
            return "#FFA81B" if self.isExplored() else "#FFF57F"
        else:
            x, y = self.pos
            cellBelief = SearcherDrosophila.sharedBeliefGrid.grid[x][y]
            if 0.33 < cellBelief <= 0.66:
                red_html = str(self.int_to_hex(int((cellBelief - 0.33) * 760)))
                green_html = "FF"
                blue_html = "00"
            elif cellBelief > 0.66:
                red_html = "FF"
                green_html = str(self.int_to_hex(
                    int(255 - ((cellBelief - 0.66) * 760))))
                blue_html = "00"
            else:   # < 0.33
                red_html = "00"
                green_html = "FF"
                blue_html = str(self.int_to_hex(255 - int(cellBelief * 760)))
            # opacity_as_str = str(self.int_to_hex(20 + int(cellBelief * 235)))
            if self.isVisible() and cellBelief < 0.90:
                return "#" + red_html + green_html + blue_html + "60"
            return "#" + red_html + green_html + blue_html + "FF"

    # A simple frontier definition : a cell which is explored
    # and that has a direct neighbor (not diagonal) which is unexplored
    # A Wall can't be a frontier

    def isFrontier(self):
        # conditions pour être une frontiere
        # une case doit ne pas contenir de mur
        # ne doit pas contenir de robot
        # avoir été explorée
        # avoir au moins un voisin inexplorée

        return (self.pos in Robot.shared_map.getFrontiersToExplore()) and (self.pos not in Robot.shared_map.getWalls())

    def int_to_hex(self, nr):
        h = format(int(nr), 'x')
        return '0' + h if len(h) % 2 else h

    # Write the value utility on the cell
    def getValue(self):
        if self.model.robot_type == "Burgard":
            if self.pos in Robot.shared_map.getEveryFrontierUtility():
                return str(round(Robot.shared_map.getFrontierCellUtility(self.pos), 1))
            else:
                return ""
        elif self.model.robot_type == "BurgardEntropy":
            if self.pos in Robot.shared_map.getEveryFrontierUtility() and Robot.shared_map.getCellEntropy(self.pos):
                if Robot.shared_map.getCellEntropy(self.pos)==0:
                    return "X"
                else:
                    return str(round(Robot.shared_map.getCellEntropy(self.pos), 1))    
            # if Robot.shared_map.getCellEntropy(self.pos):
            #     return str(round(Robot.shared_map.getCellEntropy(self.pos), 2))             
            else:
                return ""
        elif self.model.robot_type == "BurgardDrosophile":
            x, y = self.pos
            cellBelief = SearcherDrosophila.sharedBeliefGrid.grid[x][y]
            if self.pos in Robot.shared_map.getEveryFrontierUtility():
                # return str(round(Robot.shared_map.getFrontierCellUtility(self.pos)*cellBelief, 2))
                return str(round(Robot.shared_map.getFrontierCellUtility(self.pos), 2))
            else:
                return ""
        else:
            return ""


class FakeTarget(Cell):
    """
        A target cell that needs to be discovered
        For now it has no step method, it is considered immobile
        """

    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class RobotBurgardDrosophile(SearcherDrosophila):

    # sharedBeliefGrid = BeliefGrid()

    def __init__(self, unique_id, radius, model, view_angle, beta, bRadius):
        super().__init__(unique_id, radius, model, view_angle)
        self.rotation_destination_reached = True
        self.angle_rotation_total = 0
        self.destination_reached = False
        self.beta = beta
        self.bRadius = bRadius
        self.destinations_available = []
        self.beliefAbsenceThreshold = 0.0005

    def move(self):

        for i in range(self.model.width):
            for j in range(self.model.height):
                # On regarde si la cellule a une probabilité élevée et on retourne 10% de cette probabilité en tant qu'incrément
                isTarget, increase = self.sharedBeliefGrid.isPotentialTarget(
                    i, j)
                if isTarget:
                    # On cherche la cellule frontière la plus proche de cette potentielle cible
                    Min_distance = 999
                    for cell_I in range(self.model.width):
                        for cell_J in range(self.model.height):
                            cell = cell_I, cell_J
                            if cell in Robot.shared_map.getFrontiersToExplore():
                                distance = math.sqrt(
                                    (i-cell_I)*(i-cell_I)+(j-cell_J)*(j-cell_J))
                                if Min_distance >= distance:
                                    Min_distance = distance
                                    frontier = cell_I, cell_J

                    # On vérifie que la liste de frontière n'est pas vide
                    if Robot.shared_map.getFrontiersToExplore():
                        frontierUtility = Robot.shared_map.getEveryFrontierUtility().get(frontier)
                        Robot.shared_map.addUtility(
                            frontier, frontierUtility+increase)

        for visible_cell in self.getVisibleCellsProbability(self.view_angle, self.pos):
            self.sharedBeliefGrid.update(
                visible_cell, self.getObservation(visible_cell))
            config.computation_cost += 1 # Add 1 to total computation cost
        self.destination_reached = False
        # If the destination is not a frontier anymore
        if self.frontier_dest_set and self.frontier_dest not in Robot.shared_map.getFrontiersToExplore():
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the destination is the actual position
        if self.frontier_dest_set and self.frontier_dest == self.pos:
            self.frontier_dest_set = False  # Need to find another destination
            self.next_move_set = False
            self.rotation_destination_reached = True  # Want the 360 rotation
            config.computation_cost += 1 # Add 1 to total computation cost

        # If the rotation is asked
        if self.rotation_destination_reached:
            # if it's still useful to do a rotation
            if self.isRotationUseful():
                self.doFullRotation()
                config.computation_cost += 1 # Add 1 to total computation cost
            else:
                # we decide to stop the rotation
                self.rotation_destination_reached = False
                config.computation_cost += 1 # Add 1 to total computation cost

        else:
            # If the destination is not reached
            if not self.frontier_dest_set:
                # Search the destination
                self.frontier_dest = self.destinationAssignment()
                self.frontier_dest_set = True

                # Compute the trajectory
                start = time.time() # Start computation time calc
                self.trajectory = self.findOptimalTrajectory(
                    self.frontier_dest)
                stop = time.time() # End computation time calc
                config.computation_time += stop-start # Find computation time

                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot has not next move defined
            if not self.next_move_set:
                self.next_move = self.getNextMove()
                self.next_move_set = True
                config.computation_cost += 1 # Add 1 to total computation cost

            # If the robot's orientation is fine
            if self.checkOrientation():
                # If the next move cell is occupied by another robot
                if self.isNextMoveOccupied():
                    # If a robot is on its destination and next to it
                    if self.frontier_dest in Robot.shared_map.getListPositionsRobots():
                        self.next_move = self.pos
                        self.frontier_dest = None
                        self.frontier_dest_set = False
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        # set another trajectory to the same destination

                        start = time.time() # Start computation time calc
                        self.trajectory = self.findTrajectory(
                            self.frontier_dest)

                        stop = time.time() # End computation time calc
                        config.computation_time += stop-start # Find computation time

                        # set the next move
                        self.next_move = self.getNextMove()
                        config.computation_cost += 1 # Add 1 to total computation cost

                # move the robot the next move cell
                self.model.grid.move_agent(self, self.next_move)
                self.next_move_set = False
                # update the map
                self.updatePositionOnMaps()
                config.computation_cost += 1 # Add 1 to total computation cost

            if self.pos == self.rotation_destination_reached:
                self.destination_reached = True
                config.computation_cost += 1 # Add 1 to total computation cost

    def cost(self):
        """ Cost computation """
        position = self.pos
        config.computation_cost += 1 # Add 1 to total computation cost
        numberOfFrontier = len(
            Robot.shared_map.getFrontiersToExplore())  # should be local but we consider communication at anytime
        # Initialise the cost to 0 for the actual cell
        self.local_map.addCost(self.pos, 0)
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_next = set()  # Cells who will be computed
        # Cells computed in the previous iteration
        config.computation_cost += 1 # Add 1 to total computation cost

        cells_prev = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost
        # Cells already computed in all iterations
        cells_already_calculated = set([position])
        config.computation_cost += 1 # Add 1 to total computation cost

        # while there is still a frontier to explore
        while numberOfFrontier != 0 and len(cells_prev) > 0:
            # Catch the cell's neighbours
            for pos in cells_prev:
                # print(pos)
                config.computation_cost += 1 # Add 1 to total computation cost
                cells_next = cells_next.union(
                    set(self.model.grid.get_neighborhood(pos, moore=True, include_center=False)))

            # Remove the cells already computed
            cells_next -= cells_already_calculated
            config.computation_cost += 1 # Add 1 to total computation cost

            # Initialise the cost to infinity
            for cell in cells_next:
                self.local_map.addCost(cell, math.inf)
                config.computation_cost += 1 # Add 1 to total computation cost

            # Update the cost of the cells in cells_next
            for elem in cells_next:
                neighbours_list = self.model.grid.get_neighborhood(
                    elem, moore=True, include_center=False)
                # Take the neighbours of that cell
                for neighbour in neighbours_list:
                    # If this neighbour has no cost, give it
                    if neighbour not in self.local_map.getEveryCost():
                        self.local_map.addCost(neighbour, math.inf)
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # If neighbour a wall, its occupancy = 1
                    if neighbour in Robot.shared_map.getWalls():
                        occupancy_value = 1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    else:
                        occupancy_value = 0.1
                        config.computation_cost += 1 # Add 1 to total computation cost
                    # Cost comparison, the minimum is the best
                    if self.local_map.getCellCost(elem) > self.local_map.getCellCost(neighbour) + get_distance(
                            neighbour, elem) * occupancy_value:
                        config.computation_cost += 1 # Add 1 to total computation cost
                        self.local_map.addCost(elem, self.local_map.getCellCost(neighbour) + get_distance(neighbour,
                                                                                                          elem) * occupancy_value)
                # If the cell is a frontier, we reduce its number
                if elem in self.local_map.getFrontiersToExplore():
                    if self.local_map.getCellCost(elem) < math.inf:
                        numberOfFrontier -= 1
                        config.computation_cost += 1 # Add 1 to total computation cost

            cells_prev = cells_next
            cells_already_calculated = cells_already_calculated.union(
                cells_prev)
            config.computation_cost += 1 # Add 1 to total computation cost

    def destinationAssignment(self):

        self.cost()  # compute the cost

        destination = None
        destination_interest = -math.inf
        utilityCostRatio = self.getAttribute("beta")  # β
        # The quantity β ≥ 0 determines the relative importance of
        # utility versus cost

        config.computation_cost += 1 # Add 1 to total computation cost

        # Update the utility to all the frontier cells
        for frontierCell, utility in Robot.shared_map.getEveryFrontierUtility().items():
            if destination_interest < (utility * self.sharedBeliefGrid.grid[frontierCell[0]][frontierCell[1]] - utilityCostRatio * self.local_map.getCellCost(frontierCell)):
                destination_interest = (utility * self.sharedBeliefGrid.grid[frontierCell[0]][frontierCell[1]] - utilityCostRatio *
                                        self.local_map.getCellCost(frontierCell))
                destination = frontierCell
                config.computation_cost += 1 # Add 1 to total computation cost
        if destination is None:
            config.computation_cost += 1 # Add 1 to total computation cost
            return self.pos
        self.reduceUtility(destination)
        config.computation_cost += 1 # Add 1 to total computation cost

        return destination

    def reduceUtility(self, destination):
        # For the potential cells visible when the destination reached
        for hypotheticalFrontierCell in self.getCellsCoordinatesBorderRadius(destination):
            # Reduce the utility for the other robots
            if hypotheticalFrontierCell in Robot.shared_map.getEveryFrontierUtility():
                if get_distance(destination, hypotheticalFrontierCell) <= (self.getAttribute("bRadius")):

                    config.computation_cost += 1 # Add 1 to total computation cost

                    Robot.shared_map.addUtility(hypotheticalFrontierCell, (get_distance(
                        destination, hypotheticalFrontierCell) / self.getAttribute("bRadius")))
                if hypotheticalFrontierCell == destination:
                    Robot.shared_map.addUtility(hypotheticalFrontierCell, 0)

 # Pour récupérer les paramètres de Burgard pour les calculs

    def getAttribute(self, attribute):
        if attribute == "view":
            return self.view_angle
        if attribute == "beta":
            return self.beta
        if attribute == "bRadius":
            return self.bRadius

