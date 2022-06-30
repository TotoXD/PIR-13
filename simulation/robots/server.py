from mesa.visualization.modules import CanvasGrid, ChartModule, PieChartModule
from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter

from robots.model import RobotsExploration
from robots.agent import Robot, RobotRandom, RobotRandomBehaviors, RobotRandomAnt, RobotYamauchi, RobotBurgard, RobotMinPos, RobotMinPos2, SearcherDrosophila, RobotEntropy, SearcherDrosophilaShortest, Wall, Cell, Target, FakeTarget

from robots.grid_utils import verification_parameters
from robots.utils import *
from robots.agent import RobotBurgardDrosophile


set_walls, HEIGHT, WIDTH, map_name, positions_robots, positions_targets, positions_fakeTargets = verification_parameters()

if len(positions_robots) != 0:
    MAX_ROBOTS_MODEL = len(positions_robots)
else:
    MAX_ROBOTS_MODEL = MAX_ROBOTS

if len(positions_targets) != 0:
    MAX_TARGET = len(positions_targets)
else:
    MAX_TARGET = 10

model_params = {
    "Nr": UserSettableParameter("slider", "Number of Robots", 2, 1, MAX_ROBOTS_MODEL, 1),
    "Nt": UserSettableParameter("slider", "Number of Targets", 0, 0, MAX_TARGET, 1),
    "Rr": UserSettableParameter("slider", "Radius of robots", 3, 1, 10, 1),
    "Rva": UserSettableParameter("slider", "View Angle of robots", 90, 30, 360, 15),
    "Robot_type": UserSettableParameter("choice", "Robot's strategy", value=TYPE_ROBOTS[5], choices=TYPE_ROBOTS),
    "Diagonal" : UserSettableParameter("checkbox", "Déplacements diagonaux", value=False),
    "height": HEIGHT,
    "width": WIDTH,
    "walls": set_walls,
    "map_name": map_name,
    "positions_robots": positions_robots,
    "positions_targets": positions_targets,
    "positions_fakeTargets" : positions_fakeTargets,
    "random_init_positions": UserSettableParameter("checkbox", "Random Initial Position", value=False),
    "beta": UserSettableParameter("slider", "Utility-Cost ratio", 1, 0.1, 50, 0.1),
    "bRadius": UserSettableParameter("slider", "Utility radius", 3, 1, 10, 1),
}

COLORS = {"Robot": "#B0BEC5", "Wall": "#000000", "Explored_Cell": "#FFA81B", "Unexplored_Cell": "#FFF57F",
          "Target": "#FF6BF380"}


def element_portrayal(agent):
    if agent is None:
        return

    elif type(agent) is Cell or type(agent) is FakeTarget:
        portrayal = {"Shape": "rect", "w": 1,
                     "h": 1, "Filled": "true", "Layer": 0}
        x, y = agent.pos
        portrayal["x"] = x
        portrayal["y"] = y
        portrayal["Color"] = agent.getColor()
        portrayal["text"] = agent.getValue()
        portrayal["text_color"] = "#000000"

    elif type(agent) in [Robot, RobotRandom, RobotRandomBehaviors, RobotRandomAnt, RobotYamauchi, RobotBurgard, RobotEntropy, RobotMinPos, RobotMinPos2, SearcherDrosophila, SearcherDrosophilaShortest, RobotBurgardDrosophile]:
        portrayal = {"Shape": "circle", "r": 1, "Filled": "false", "Layer": 2, "text": agent.unique_id,
                     "text_color": "#000000"}
        x, y = agent.pos
        portrayal["x"] = x
        portrayal["y"] = y
        portrayal["Color"] = COLORS["Robot"]

    elif type(agent) is Wall:
        portrayal = {"Shape": "rect", "w": 1,
                     "h": 1, "Filled": "true", "Layer": 1}
        x, y = agent.pos
        portrayal["x"] = x
        portrayal["y"] = y
        portrayal["Color"] = agent.getColor()

    elif type(agent) is Target:
        portrayal = {"Shape": "circle", "r": 1, "Filled": "false",
                     "Layer": 1, "text": "X", "text_color": "#000000"}
        x, y = agent.pos
        portrayal["x"] = x
        portrayal["y"] = y
        portrayal["Color"] = COLORS["Target"]

    return portrayal


# CanvasGrid(portrayal_method, grid_width, grid_height, canvas_width, canvas_height)
canvas_element = CanvasGrid(element_portrayal, WIDTH, HEIGHT, 500, 500)

chart_element1 = ChartModule(
    [{"Label": "Explored Area Percentage", "Color": "#AA0000"}]
)

chart_element2 = ChartModule(
    [{"Label": "Exploration Cost", "Color": "#00aa00"}]
)

chart_element3 = ChartModule(
    [{"Label": "Number of movement", "Color": "#00aaaa"}]
)

chart_element4 = ChartModule(
    [{"Label": "Rotation Cost", "Color": "#B40A0B"}]
)

chart_element5 = ChartModule(
    [{"Label": "Number of Rotation", "Color": "#B40A0B"}]
)

chart_element6 = ChartModule(
    [{"Label": "Efficiency", "Color": "#B40A0B"}]
)

chart_element7 = ChartModule(
    [{"Label": "Computation Time", "Color": "#B40A0B"}]
)

chart_element8 = ChartModule(
    [{"Label": "Computation Cost", "Color": "#B40A0B"}]
)

chart_element9 = ChartModule(
    [{"Label": "Mean Belief", "Color": "#B40A0B"}]
)


server = ModularServer(
    RobotsExploration, [canvas_element, chart_element1,
                        chart_element2, chart_element3, chart_element4, chart_element5, chart_element6, chart_element7, chart_element8, chart_element9],
    "Model Multi-robots Exploration",
    model_params
)

server.port = 8521
