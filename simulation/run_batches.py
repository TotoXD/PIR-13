# run_batches permet d'executer plusieurs simulations à la suite

from robots.model import *
from robots.grid_utils import verification_parameters
from robots.utils import *
from mesa.batchrunner import BatchRunner

# verifications des paramètres
set_walls, HEIGHT, WIDTH, map_name, positions_robots, positions_targets, position_fake_targets = verification_parameters()

if len(positions_robots) != 0:
    MAX_ROBOTS_MODEL = len(positions_robots)
else:
    MAX_ROBOTS_MODEL = 7

if len(positions_targets) != 0:
    MAX_TARGET = len(positions_targets)
else:
    MAX_TARGET = 10

# Paramètres fixes durant l'execution des simu
fixed_params = {"width": WIDTH,
                "height": HEIGHT,
                "walls": set_walls,
                "Rr": 5,
                "Nt": 1,
                "positions_robots": positions_robots,
                "positions_targets": positions_targets,
                # "position_fake_targets" : position_fake_targets,
                "map_name": map_name,
                "Rva": 90,
                "random_init_positions": False,
                "beta": 10,
                "bRadius": 5,
                }

# Paramètres variables qui vont être analysés
variable_params = {
    "Nr": range(8, 9, 1),
    # "Rr": range(3, 8, 2),
    # "bRadius":range(1, 9, 1),
    "Robot_type": ["Burgard"],
}

# Paramètres des simulations 
batch_run = BatchRunner(RobotsExploration,
                        variable_params,
                        fixed_params,
                        iterations=20,
                        max_steps=1000,
                        model_reporters={"step_number_end_exploration": last_step_exploration})

# Lancement
batch_run.run_all()
