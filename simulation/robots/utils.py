# utils contient différentes fonctions pour gérer les résultats, écrire dans les fichiers...

import os
from datetime import date
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import csv
from os import path

# Types de robots
TYPE_ROBOTS = ["Random", "RandomBehaviors", "RandomAnt", "Yamauchi", "Burgard", "BurgardEntropy", "MinPos", "MinPos2",
               "SearcherDrosophila", "SearcherDrosophilaShortest", "BurgardDrosophile"]
TYPE_ROBOTS_BATCHES = ["Yamauchi", "Burgard","Random", "RandomBehaviors", "RandomAnt", "Yamauchi", "Burgard", "BurgardEntropy", "MinPos", "MinPos2",
               "SearcherDrosophila", "SearcherDrosophilaShortest", "BurgardDrosophile"]

# Différents paramètres de simulation
INFOS_ASKED = [("Date of simulation: ", "str"),
               ("# of steps to explore: ", "int"),
               ("# of robots: ", "int"),
               ("Type of robots: ", "str"),
               ("Radius of robots: ", "int"),
               ("Position predefined: ", "str"),
               ("Name of the map: ", "str"),
               ("Height: ", "int"),
               ("Width: ", "int"),
               ("# of targets: ", "int"),
               ("# of cells to discover: ", "int"),
               ("Efficiency: ", "int"),
               ("Exploration cost : ", "float"),
               ("Number of movement: ", "int"),
               ("Rotation Cost : ", "float"),
               ("Mean Belief : ", "float"),
               ("Number of Rotation: ", "int"),
               ("Total cost : ", "int"),
               ("Computation cost : ", "int"),
               ("Beta : ", "int"),
               ("Radius Utility: ", "int"),]

FIELDNAMES = [
    "map_name",
    "number_robots",
    "radius_robots",
    "type_robots",
    "num_steps",
    "number_targets",
    "positions_predefined",
    "efficiency",
    "exploration_cost",
    "number_cells",
    "map_height",
    "map_width",
    "rotation",
    "rotation_cost",
    "total_cost",
    "computation_cost",
    "calcul_time",
    "beta",
    "bRadius",
    "belief",
]

MAX_ROBOTS = 10

# Fonctions utils

def getNextNumberDirectory(path_dir='results/'):
    list_dir = os.listdir(path_dir)
    num_dir = 0

    if len(list_dir) != 0:
        for name in list_dir:
            if int(name) > num_dir:
                num_dir = int(name)

    return num_dir + 1


def getColor(index):
    colors = list(mcolors.TABLEAU_COLORS.items())
    _, rgb = colors[index % len(colors)]

    return rgb


def createDirectory(new_dir_name, path_dir='results/'):
    os.mkdir(path_dir+new_dir_name)
    return path_dir+new_dir_name+"/"


def saveTextFile(content, path_dir='results/', file_name="informations.txt"):
    f = open(path_dir+file_name, "w")
    f.write(content)
    f.close()


def fileResultsCSVExists(path_file='results/results.csv'):
    return path.exists(path_file)


def addResultsToCSV(dict_infos, path_dir='results/', file_name="results.csv"):
    path_file = path_dir+file_name

    # si le fichier .csv n'existe pas on le créé et on ajoute les noms des colonnes
    if not fileResultsCSVExists(path_file):
        with open(path_file, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=FIELDNAMES)
            writer.writeheader()

    with open(path_file, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=FIELDNAMES)
        writer.writerow(dict_infos)


def createContentFromInformations(dict_infos): # Pour ecrire dans results.csv
    chaine = ""
    chaine += "{1}{0}\n".format(date.today().strftime("%d/%m/%Y"),
                                INFOS_ASKED[0][0])
    chaine += "{1}{0}\n".format(dict_infos.get("num_steps",
                                "NA"), INFOS_ASKED[1][0])

    chaine += "\nROBOTS\n{1}{0}\n".format(
        dict_infos.get("number_robots", "NA"), INFOS_ASKED[2][0])
    chaine += "{1}{0}\n".format(dict_infos.get("radius_robots",
                                "NA"), INFOS_ASKED[4][0])
    chaine += "{1}{0}\n".format(dict_infos.get("type_robots",
                                "NA"), INFOS_ASKED[3][0])
    chaine += "{1}{0}\n".format(dict_infos.get("positions_predefined",
                                "NA"), INFOS_ASKED[5][0])
    chaine += "\nMAP\n{1}{0}\n".format(
        dict_infos.get("map_name", "NA"), INFOS_ASKED[6][0])
    chaine += "{1}{0}\n".format(dict_infos.get("map_height",
                                "NA"), INFOS_ASKED[7][0])
    chaine += "{1}{0}\n".format(dict_infos.get("map_width",
                                "NA"), INFOS_ASKED[8][0])
    chaine += "{1}{0}\n".format(dict_infos.get("number_targets",
                                "NA"), INFOS_ASKED[9][0])
    chaine += "{1}{0}\n".format(dict_infos.get("number_cells",
                                "NA"), INFOS_ASKED[10][0])
    chaine += "{1}{0}\n".format(dict_infos.get("efficiency",
                                "NA"), INFOS_ASKED[11][0])
    chaine += "{1}{0}\n".format(dict_infos.get("exploration_cost",
                                "NA"), INFOS_ASKED[12][0])
    chaine += "{1}{0}\n".format(dict_infos.get("beta",
                                "NA"), INFOS_ASKED[13][0])
    chaine += "{1}{0}\n".format(dict_infos.get("bRadius",
                                "NA"), INFOS_ASKED[14][0])
    chaine += "{1}{0}\n".format(dict_infos.get("total_cost",
                               "NA"), INFOS_ASKED[15][0])
    chaine += "{1}{0}\n".format(dict_infos.get("calcul_time",
                            "NA"), INFOS_ASKED[16][0])
    return chaine


def createDirectoriesAndFiles(dict_infos, map_name, path_dir_init="results/"):
    # mettre en forme le map_name pour enlever l'extension et le "results/grids/"
    slash_split = map_name.split("/")
    map_with_extension = slash_split[-1]
    map_name, _ = map_with_extension.split(".")
    path_dir = path_dir_init

    # modifier nom de la carte dans le dict infos
    dict_infos["map_name"] = map_name

    # tester si le dossier associé à la map existe déjà sinon le créer
    if not os.path.isdir(path_dir_init + map_name):
        path_dir = createDirectory(map_name, path_dir_init)
    else:
        path_dir += map_name + "/"

    # récupérer le chemin du nouveau dossier
    str_folder_name = str(getNextNumberDirectory(path_dir))

    # créer le dossier en question
    path_dir = createDirectory(str_folder_name, path_dir)

    # créer le contenu
    content = createContentFromInformations(dict_infos)
    # le sauvegarder dans un fichier dédié
    saveTextFile(content, path_dir)
    addResultsToCSV(dict_infos, path_dir_init)

    return path_dir
