from mesa import Model
from mesa.datacollection import DataCollector
from mesa.space import MultiGrid
from mesa.time import RandomActivation

from robots.agent import FakeTarget, Robot, RobotRandom, RobotRandomBehaviors, RobotRandomAnt, RobotYamauchi, RobotBurgard, RobotEntropy, RobotMinPos, RobotMinPos2, SearcherDrosophila, \
    SearcherDrosophilaShortest, Wall, Cell, Target, RobotBurgardDrosophile
from robots.MapRobot import *
from robots.utils import *
from matplotlib.patches import Rectangle

# import numpy as np
import random
import math
from . import config


def compute_discovered(model):
    percentage_discovered = len(
        Robot.shared_map.getCellsExplored()) * 100 / (model.num_cells_to_discover)
    return percentage_discovered


def last_step_exploration(model):
    return model.schedule.steps


def exploration_cost(model):
    total_steps = 0
    for robot in model.schedule.agent_buffer(shuffled=True):
        total_steps += robot.model.schedule.steps
    return total_steps


def efficiency(model):
    if exploration_cost(model) != 0:
        return len(Robot.shared_map.getCellsExplored()) / total_cost(model)
    else:
        return 0


def rotation(model):
    return(config.rotation_cost)

def rotation_angle(model):
    return(config.rotation_cost_angle)

def total_cost(model):
    return(exploration_cost(model)-rotation(model)+rotation_angle(model))


def computation_cost(model):
    return(config.computation_cost)

def timer(model):
    return(config.computation_time)

def belief(model):
    return(config.belief)


class RobotsExploration(Model):
    """
    A simple multi-robots exploration model
    """

    def __init__(self, Nr=4, Nt=1, Rr=2, Rva=360, Robot_type="Random",Diagonal=True, width=10, height=10, walls=None, map_name="NA",
                 positions_robots=[], positions_targets=[],positions_fakeTargets=[], random_init_positions=True, beta=1, bRadius=3):

        super().__init__()

        print(
            "Paramètres variables : Beta: {0} / Radius Utility: {1}".format(beta, bRadius))

        self.set_walls = walls

        self.width = width
        self.height = height
        self.robot_type = Robot_type
        self.diagonal=Diagonal
        self.num_robots = Nr
        self.num_targets = Nt
        self.robotRadius = Rr
        self.robotViewAngle = Rva
        self.map_name = map_name
        self.robot_type = Robot_type
        self.beta = beta
        self.bRadius = bRadius

        # contient les cellules dans le champ de vision des robots à chaque itération : permet leur affichage
        self.cells_visible = set()

        self.running = True
        positions_predefined = "False"

        self.list_robots = []
        # Reset des variables partagées entre chaque simulation
        config.rotation_cost = 0
        config.computation_cost = 0
        config.rotation_cost_angle = 0
        config.computation_time = 0
        config.belief=0
        # No agent limit per cell
        # torus means : appearing on the other side of the grid
        # if the robot leaves the grid by one side
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = RandomActivation(self)

        # creating each individual agent Cell
        for xc in range(self.grid.width):
            for yc in range(self.grid.height):
                c = Cell(self.next_id(), self)
                self.grid.place_agent(c, (xc, yc))

        # placing the walls
        for xw, yw in self.set_walls:
            w = Wall(self.next_id(), self)
            self.grid.place_agent(w, (xw, yw))

        if not random_init_positions:
            positions_predefined = "True"

        # reset the global map shared between robots
        Robot.shared_map = GlobalMap(self.num_robots)
        SearcherDrosophila.sharedBeliefGrid = BeliefGrid(
            self.width, self.height, positions_targets,positions_fakeTargets)

        # create the agents
        positions_init_robots = []
        for i in range(self.num_robots):
            if random_init_positions:
                # calculate random positions
                x, y = self.define_new_coordinates(positions_init_robots)
            else:
                # tant qu'on a de la place disponible on ajoute les robots sur les positions prédéfinies
                if i < len(positions_robots):
                    x, y = positions_robots[i]
                else:
                    # si on a plus de robots que de positions prédéfinies, on tire les dernière de manière aléatoire
                    x, y = self.define_new_coordinates(positions_init_robots)

            # careful, place_agent don't take in account the fact
            # that a cell might not be free
            if self.robot_type == "Random":
                r = RobotRandom(i, self.robotRadius, self, Rva, rotation_angle=45,
                                proba_move=66, proba_turn_right=50)
            elif self.robot_type == "RandomBehaviors":
                r = RobotRandomBehaviors(
                    i, self.robotRadius, self, Rva, rotation_angle=45, proba_move_routine=75, proba_move_standard=50, proba_move_anxious=25)
            elif self.robot_type == "RandomAnt":
                r = RobotRandomAnt(
                    i, self.robotRadius, self, Rva, rotation_angle=45, min_times_moving_forward=2, max_times_moving_forward=8)
            elif self.robot_type == "Yamauchi":
                r = RobotYamauchi(i, self.robotRadius, self, Rva)
            elif self.robot_type == "Burgard":
                r = RobotBurgard(i, self.robotRadius, self,
                                 Rva, beta, bRadius)
            elif self.robot_type == "BurgardEntropy":
                r = RobotEntropy(i, self.robotRadius, self,
                                 Rva, beta, bRadius)
            elif self.robot_type == "MinPos":
                r = RobotMinPos(unique_id=i, radius=self.robotRadius,
                                model=self, view_angle=Rva)
            elif self.robot_type == "MinPos2":
                r = RobotMinPos2(unique_id=i, radius=self.robotRadius,
                                model=self, view_angle=Rva)
            elif Robot_type == "SearcherDrosophila":
                r = SearcherDrosophila(i, self.robotRadius, self, Rva)
            elif Robot_type == "SearcherDrosophilaShortest":
                r = SearcherDrosophilaShortest(i, self.robotRadius, self, Rva)
            elif self.robot_type == "BurgardDrosophile":
                r = RobotBurgardDrosophile(
                    i, self.robotRadius, self, Rva, beta, bRadius,)

            self.schedule.add(r)
            self.grid.place_agent(r, (x, y))
            positions_init_robots.append((x, y))

            Robot.shared_map.addCellExplored(r.pos, r.unique_id)
            r.updateLocalMapCoordinates()

            # make the robots look around themselves
            self.list_robots.append(r)

        for r in self.list_robots:
            Robot.shared_map.updatePositionRobot(r.unique_id, r.pos)
            r.robotVision()
            Robot.shared_map.updatePositionRobot(r.unique_id, r.pos)

        # determine the frontiers
        Robot.shared_map.determineFrontiers()

        # placing the targets
        define_random_positions_targets = True
        if (positions_targets != [] and self.num_targets != 0):
            self.num_targets = len(positions_targets)
            define_random_positions_targets = False

        for i in range(self.num_targets):
            if define_random_positions_targets:
                # calculate random positions
                xt, yt = self.define_new_coordinates(positions_init_robots)

            else:
                xt, yt = positions_targets[i]
            t = Target(self.next_id(), self)
            self.grid.place_agent(t, (xt, yt))

        # Placing FakeTargets
        for i in range(0,len(positions_fakeTargets)):
            xt, yt = positions_fakeTargets[i]
            Ft = FakeTarget(self.next_id(), self)
            self.grid.place_agent(Ft, (xt, yt))


        # WARN: not accurate for now
        self.num_cells_to_discover = self.grid.height * \
            self.grid.width - len(self.set_walls)
        # data collector
        self.datacollector = DataCollector(
            {
                "Explored Area Percentage": compute_discovered,
                "step_number_end_exploration": last_step_exploration,
                "Number of movement": exploration_cost,
                "Efficiency": efficiency,
                "Number of Rotation": rotation,
                "Exploration Cost": total_cost,
                "Computation Cost": computation_cost,
                "Rotation Cost":rotation_angle,
                "Computation Time":timer,
                "Mean Belief":belief
            }
        )

        self.verbose = False

        self.dict_infos = {
            "number_robots": self.num_robots,
            "radius_robots": self.robotRadius,
            "map_height": self.height,
            "map_width": self.width,
            "number_targets": self.num_targets,
            "type_robots": self.robot_type,
            "map_name": map_name,
            "number_cells": self.num_cells_to_discover,
            "positions_predefined": positions_predefined,
            "efficiency": 0,
            "exploration_cost": 0,
            "rotation": 0,
            "total_cost": 0,
            "computation_cost": 0,
            "rotation_cost":0,
            "calcul_time":0,
            "belief":0,
            "beta": self.beta,
            "bRadius": self.bRadius,
        }

    def define_new_coordinates(self, list_init_robots):
        """
        Check if the cell is free or not to avoid to throw an Exception
        """
        x = self.random.randrange(self.grid.width)
        y = self.random.randrange(self.grid.height)

        # We check that there is no Robot or Wall in this cell
        contents = self.grid.get_cell_list_contents([(x, y)])
        for agents in contents:
            if type(agents) is Wall or (x, y) in list_init_robots or type(agents) is Target:
                x, y = self.define_new_coordinates(list_init_robots)
        return x, y

    def step(self):
        # On vide l'ensemble des cellules visibles à chaque itération
        self.cells_visible = set()
        for frontierCell in Robot.shared_map.getFrontiersToExplore():  # set utility to 1 for every frontier cell
            Robot.shared_map.addUtility(frontierCell, 1)
        if self.robot_type == "BurgardEntropy":
            self.list_robots[0].determineOccupancy()
            self.list_robots[0].determineEntropy()
            self.list_robots[0].determineEntropyFrontiers()
        if self.robot_type == "Burgard" or self.robot_type == "BurgardDrosophile" or self.robot_type == "BurgardEntropy":
            for r in self.list_robots:
                if not r.destination_reached and r.frontier_dest is not None and r.frontier_dest in Robot.shared_map.getFrontiersToExplore():
                    # r.shared_map.addEntropy(r.frontier_dest, 0)
                    r.reduceUtility(r.frontier_dest)
                    # r.shared_map.addEntropy(r.frontier_dest, 0)
        self.datacollector.collect(self)
        self.schedule.step()

        for r in self.list_robots:
            if r.isExplorationOver():
                self.running = False
                self.dict_infos["num_steps"] = self.schedule.steps + 1
                self.dict_infos["efficiency"] = efficiency(self)
                self.dict_infos["exploration_cost"] = exploration_cost(self)
                self.dict_infos["rotation"] = rotation(self)
                self.dict_infos["total_cost"] = total_cost(self)
                self.dict_infos["computation_cost"] = computation_cost(self)
                self.dict_infos["rotation_cost"] = rotation_angle(self)
                self.dict_infos["calcul_time"]=timer(self)
                self.dict_infos["belief"]=belief(self)

                path_dir = createDirectoriesAndFiles(
                    self.dict_infos, self.map_name)

                # sauvegarder le graphique
                self.createCollaborativeMap(self.list_robots, path_dir)

                self.compareLocalMaps(self.list_robots, path_dir)
                break

    def createCollaborativeMap(self, list_robots, path_dir, file_name="collaborative_map.png"):
        fig = plt.figure(2)
        fig.clf()
        ax = fig.add_subplot(111)

        for robot in list_robots:
            color = getColor(robot.unique_id)

            plt.scatter(*zip(*Robot.shared_map.getCellByDiscoverer(robot.unique_id)), c=color, marker="s",
                        label="Robot {0}".format(robot.unique_id))
            for c_x, c_y in Robot.shared_map.getCellByDiscoverer(robot.unique_id):
                ax.add_patch(Rectangle(xy=(c_x - 0.5, c_y - 0.5),
                             width=0.9, height=0.9, color=color, fill=True))

        if len(Robot.shared_map.getWalls()) > 0:
            plt.scatter(*zip(*list(Robot.shared_map.getWalls())),
                        c="black", marker="s", label="Walls")
            for w_x, w_y in Robot.shared_map.getWalls():
                ax.add_patch(Rectangle(xy=(w_x - 0.5, w_y - 0.5),
                             width=0.9, height=0.9, color="black", fill=True))

        plt.title(
            "Map representing the contribution of each robot ({0}) in the exploration process".format(self.robot_type))
        # plt.grid()
        plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1))

        if self.height == self.width:
            ax.set_aspect('equal', adjustable='box')

        fig.savefig(path_dir+file_name, dpi=250, bbox_inches='tight')

        if self.verbose:
            plt.show()

        return fig

    def compareLocalMaps(self, list_robots, path_dir, file_name="local_maps.png"):
        fig = plt.figure(1, constrained_layout=True)
        fig.clf()

        size = math.ceil(math.sqrt(self.num_robots))

        full_graph = fig.add_gridspec(size, size)  # 2 rows and 2 columns

        for robot in list_robots:
            color = getColor(robot.unique_id)

            scatter_r = fig.add_subplot(
                full_graph[robot.unique_id // size, robot.unique_id % size])
            scatter_r.scatter(*zip(*robot.local_map.getCellsExplored()), c=color, marker="s",
                              label="Robot {0}".format(robot.unique_id))
            for c_x, c_y in robot.local_map.getCellsExplored():
                scatter_r.add_patch(Rectangle(
                    xy=(c_x - 0.5, c_y - 0.5), width=0.9, height=0.9, color=color, fill=True))

            if self.height == self.width:
                scatter_r.set_aspect('equal', adjustable='box')
            scatter_r.set_title(
                "Local map of Robot {0}".format(robot.unique_id))

            if len(Robot.shared_map.getWalls()) > 0:
                scatter_r.scatter(
                    *zip(*list(robot.local_map.getWalls())), c="black", marker="s", label="Walls")
                for w_x, w_y in robot.local_map.getWalls():
                    scatter_r.add_patch(Rectangle(
                        xy=(w_x - 0.5, w_y - 0.5), width=0.9, height=0.9, color="black", fill=True))

            plt.xlim(0, self.width-1)
            plt.ylim(0, self.height-1)

        # or plt.suptitle('Main title')
        fig.suptitle('Comparison of the different local maps')
        fig.savefig(path_dir+file_name, dpi=250, bbox_inches='tight')

        if self.verbose:
            plt.show()

        return fig
