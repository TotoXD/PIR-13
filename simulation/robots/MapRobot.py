import matplotlib.pyplot as plt
import random
import math
from . import config
import time

class MapRobot():

    def __init__(self, iterable=[]):
        self.cells_explored = set()
        self.all_cells = set()
        self.frontiers_to_explore = set()
        self.walls = set(iterable)
        self.target = set(iterable)
        self.costGrid = dict()
        self.utilityGrid = dict()
        self.entropyGrid = dict()
        self.occupancyGrid = dict()

    def __repr__(self):
        return "__repr__"

    def __str__(self):
        return str(self.frontiers_to_explore)

    def getCellsExplored(self):
        return self.cells_explored

    def getTarget(self):
        return self.target

    def getFrontiersToExplore(self):
        return self.frontiers_to_explore

    def getEveryCost(self):
        return self.costGrid

    def getAlreadySeen(self):
        return self.cells_explored.union(self.walls)

    def getCellCost(self, cell):
        return self.costGrid.get(cell)

    def getEveryFrontierUtility(self):
        return self.utilityGrid

    def getEveryFrontierEntropy(self):
        return self.entropyGrid

    def getFrontierCellUtility(self, cell):
        return self.utilityGrid.get(cell)

    def getEveryCellEntropy(self):
        return self.entropyGrid

    def getCellEntropy(self, cell):
        # cell is a tuple
        return self.entropyGrid.get(cell)

    def getCellOccupancy(self, cell):
        # cell is a tuple
        return self.occupancyGrid.get(cell)

    def getWalls(self):
        return self.walls

    def displayMap(self, title):
        # plot the walls
        plt.scatter(*zip(*list(self.getWalls())), c="black", marker="s")
        plt.title(title)
        plt.grid(True)

        plt.show()

    def dist(self, cell1, cell2):
        # Calcul distance
        x, y = cell1
        cx, cy = cell2

        x_dist = (x - cx) ** 2
        y_dist = (y - cy) ** 2

        dist = math.sqrt(x_dist + y_dist)

        return dist

    def clusterKMean(self):
        # Cluster K-Mean
        start = time.time()

        frontiers=list(self.getFrontiersToExplore())
        print(frontiers)
        number_frontiers=len(frontiers)
        number_robots=len(self.getListPositionsRobots())
        new_centro=[]
        u=number_frontiers
        if number_robots<number_frontiers:
            u=number_robots
        for j in range(u):
            new_centro.append(())
        liste_centroide=[]

        for i in range(number_robots):
            if number_frontiers>0:
                centroide=[]
                centroide.append(frontiers.pop(random.randint(0,number_frontiers-1)))
                number_frontiers=len(frontiers)
                liste_centroide.append(centroide)
                new_centro[i]=centroide
                
        for h in range(3):

            # On attribue les frontières au centroide le plus proche
            for frontier in frontiers:
                min = -1
                centroide_index = -1
                for k in range(len(new_centro)):
                    dist = self.dist(frontier,new_centro[k][0])
                    # Si la distance est plus petite, ou alors que aucun centroide n'avait encore la frontière
                    if min == -1 or dist < min:
                        min = dist
                        centroide_index = k
                
                liste_centroide[centroide_index].append(frontier)
            # Test pour trouver la meilleure frontière parmi un groupe de frontières le plus proche au hasard -> Trouver le centre
            for i in range(len(liste_centroide)):
                index_centroide = -1
                best_frontier_index = -1
                xmean=0
                ymean=0
                # On fait la moyenne de position des points d'un cluster
                for frontier in liste_centroide[i]:
                    best_dist = -1
                    xmean+=frontier[0]
                    ymean+=frontier[1]
                xmean=xmean/len(liste_centroide[i])
                ymean=ymean/len(liste_centroide[i])
                center=(xmean,ymean)
                # On cherche le point le plus proche de cette moyenne
                for frontier in liste_centroide[i]:
                    dist=self.dist(frontier,center)
                    index_centroide+=1
                    if best_frontier_index== -1 or dist<best_dist:
                        best_dist=dist
                        best_frontier_index=index_centroide
                        new_centro[i]=[frontier]
                        
                
                print("Milieu trouvé avec le centroide ", (liste_centroide[i])[0], " : ", (liste_centroide[i])[best_frontier_index])

            # On vide la liste contenant les frontières en ne laissant que les centroide actuel
                liste_centroide[i]=new_centro[i]        
                #print(liste_centroide)
                #print("new centro",new_centro)
                #print("frontier",self.frontiers_to_explore)
        l=set()
        for i in range(len(new_centro)):
            l.add((new_centro[i])[0])
        self.frontiers_to_explore=l
        stop = time.time()
        config.computation_time += stop-start
            
    
    def clusterDBSCAN(self, radius = 3):
        # Cluster in a radius

        frontiers=list(self.getFrontiersToExplore())
        print("Frontières: ", frontiers)

        number_frontiers=len(frontiers)
        number_robots=len(self.getListPositionsRobots())
        liste_centroide=[]
        for i in range(number_robots):
            number_frontiers=len(frontiers)
            if number_frontiers > 0:
                centroide=[]
                new_centroide = frontiers.pop(random.randint(0,number_frontiers-1))
                (x_centroide, y_centroide) = new_centroide
            
                for centroide in liste_centroide:
                        x, y = centroide
                        x_cond = False
                        y_cond = False
                        # On ne traite pas le centroide lui même
                        if (x, y) != (x_centroide, y_centroide):
                            # On essaie de voir si le centroide testé est dans la portée x du nouveau centroide
                            if x_centroide <= x:
                                if (x_centroide + radius) >= x:
                                    x_cond = True
                            elif (x_centroide - radius) <= x:
                                x_cond = True
                            # On essaie de voir si le centroide testé est dans la portée y du nouveau centroide
                            if y_centroide <= y:
                                if (y_centroide + radius) >= y:
                                    y_cond = True
                            elif (y_centroide - radius) <= y:
                                y_cond = True

                            # Si le centroide est dans le rayon, on le redésigne comme frontière
                            if x_cond and y_cond:
                                i-=1
                                frontiers.append(new_centroide)

                            # Sinon on l'ajoute à la liste des centroides
                            else:
                                centroide.append(new_centroide)
                                number_frontiers=len(frontiers)
                                liste_centroide.append(centroide)

        liste_centroides_visites = []
        frontiers_left = False
        # Tant qu'il reste des frontières à attribuer
        while len(frontiers) > 0:
            number_frontiers = len(frontiers)

            # S'il reste des frontières non attribuées, qui ne sont pas dans le rayon des centroides
            if frontiers_left:
                centroide=[]
                centroide.append(frontiers.pop(random.randint(0, number_frontiers-1)))
                liste_centroide.append(centroide)

            # Pour tous les centroides
            for i in range(len(liste_centroide)):
                # Si on n'a pas encore visité le centroide
                if (liste_centroide[i])[0] not in liste_centroides_visites:
                    # Le centroide est considéré comme visité
                    liste_centroides_visites.append((liste_centroide[i])[0])
                    x_centroide, y_centroide = (liste_centroide[i])[0]
                    # Pour toutes les frontières, en partant des plus proches du centroide
                    for frontier in frontiers:
                        x, y = frontier
                        x_cond = False
                        y_cond = False
                        # On ne traite pas le centroide lui même
                        if (x, y) != (x_centroide, y_centroide):
                            # On essaie de voir si la frontière est dans la portée x du centroide
                            if x_centroide <= x:
                                if (x_centroide + radius) >= x:
                                    x_cond = True
                            elif (x_centroide - radius) <= x:
                                x_cond = True
                            # On essaie de voir si la frontière est dans la portée y du centroide
                            if y_centroide <= y:
                                if (y_centroide + radius) >= y:
                                    y_cond = True
                            elif (y_centroide - radius) <= y:
                                y_cond = True

                            # Si la frontière est dans le rayon, on l'ajoute au cluster
                            if x_cond and y_cond:
                                print("J'attribue")
                                liste_centroide[i].append(frontier)
                                frontiers.remove(frontier)
                            else:
                                break

            # S'il reste des frontières, la boucle va se réexecuter, le premier if sera executé et va donc refaire des centroïdes
            frontiers_left = True

        print("Liste centroides : ", liste_centroide)
        l=set()
        for i in range(len(liste_centroide)):
            l.add((liste_centroide[i])[0])
        self.frontiers_to_explore = l


    def findClosestFrontier(self, pos_init, num_robot=0):
        frontier_found = False
        can_propagate = True

        total_propag = set()
        prec_wave = set([pos_init])
        dict_paths = {}

        wave_number = 1
        frontier = pos_init

        trajectoire = []

        while (not frontier_found) and can_propagate:

            next_wave = set()
            total_propag = total_propag.union(prec_wave)

            # récupérer les coordonnées des voisins accessibles
            for cell in prec_wave:
                # cases ne contenant ni robot, n'étant pas un mur et ayant été découvertes
                accessible_neighbors = self.getAccessibleNeighbors(cell)

                # print("Voisins accessibles depuis {0}:".format(cell), sorted(accessible_neighbors))

                # ajout de ces voisins à l'ensemble des voisins potentiels de la vague suivante
                local_next_wave = set(accessible_neighbors)
                # print("Next wave potentielle:", sorted(local_next_wave))

                # si vague déjà propagée sur la case, on enlève la case en question
                local_next_wave -= total_propag
                # print("Next wave sans cases déjà propagées:", sorted(local_next_wave))

                # on propage la vague sur les cases restantes accessibles
                for cell_next in local_next_wave:
                    # on regarde si la case suivante a déjà des précédesseurs connus
                    # sinon on créé un ensemble vide
                    set_prec = dict_paths.get(cell_next, set())

                    # on ajoute à cet ensemble vide notre case précédente, ici il s'agit de "cell"
                    set_prec.add(cell)

                    # on stocke dans le dictionnaire à la clé "cell_next" notre ensemble
                    dict_paths[cell_next] = set_prec

                # print("Cellules précédentes de {0}: {1}".format(cell_next, dict_paths[cell_next]))

                # ajout de cette mini-vague à la prochaine vague complète
                next_wave = next_wave.union(local_next_wave)

            # print("Wave number ({1}) for {0}:".format(num_robot, wave_number), sorted(next_wave))

            # on vérifie pour chaque cell de la next wave s'il s'agit d'une frontière ou pas
            for cell in next_wave:
                if cell in self.getFrontiersToExplore():
                    # aka next_move
                    frontier = cell
                    next_move = frontier
                    frontier_found = True

                    # récupérer tout le trajet pour y accéder
                    trajectoire.append(next_move)
                    # récupérer la case suivante à accéder
                    while (pos_init not in dict_paths[next_move]):
                        # on prend au hasard une des cases de l'ensemble
                        next_move = dict_paths[next_move].pop()
                        trajectoire.append(next_move)
                    break

            if len(next_wave) == 0:
                can_propagate = False
            # print("Can't propagate wave anymore")

            else:
                prec_wave = next_wave

            wave_number += 1

        return frontier, trajectoire

    def WPA(self, pos_init, coord_robots,moore=True, destination=None, num_robot=0):
        dest_reached = False
        can_propagate = True

        total_propag = set()
        prec_wave = set([pos_init])
        dict_paths = {}

        wave_number = 1
        next_move = pos_init

        trajectoire = []

        # print("\nDestination Robot {0}: {1}".format(num_robot, destination))

        while (not dest_reached) and can_propagate:

            next_wave = set()
            total_propag = total_propag.union(prec_wave)

            # récupérer les coordonnées des voisins accessibles
            for cell in prec_wave:
                # cases ne contenant ni robot, n'étant pas un mur et ayant été découvertes
                accessible_neighbors = self.getAccessibleNeighbors(cell,moore,coord_robots)

                # print("Voisins accessibles depuis {0}:".format(cell), sorted(accessible_neighbors))

                # ajout de ces voisins à l'ensemble des voisins potentiels de la vague suivante
                local_next_wave = set(accessible_neighbors)
                # print("Next wave potentielle:", sorted(local_next_wave))

                # si vague déjà propagée sur la case, on enlève la case en question
                local_next_wave -= total_propag
                # print("Next wave sans cases déjà propagées:", sorted(local_next_wave))

                # on propage la vague sur les cases restantes accessibles
                for cell_next in local_next_wave:
                    # on regarde si la case suivante a déjà des précédesseurs connus
                    # sinon on créé un ensemble vide
                    set_prec = dict_paths.get(cell_next, set())

                    # on ajoute à cet ensemble vide notre case précédente, ici il s'agit de "cell"
                    set_prec.add(cell)

                    # on stocke dans le dictionnaire à la clé "cell_next" notre ensemble
                    dict_paths[cell_next] = set_prec

                # print("Cellules précédentes de {0}: {1}".format(cell_next, dict_paths[cell_next]))

                # ajout de cette mini-vague à la prochaine vague complète
                next_wave = next_wave.union(local_next_wave)

            # print("Wave number ({1}) for {0}:".format(num_robot, wave_number), sorted(next_wave))

            # si la destination fait partie de ces cases, destination trouvée
            if (destination in next_wave):
                next_move = destination
                trajectoire.append(destination)

                # récupérer la case suivante à accéder
                while (pos_init not in dict_paths[next_move]):
                    # on prend au hasard une des cases de l'ensemble
                    next_move = dict_paths[next_move].pop()
                    trajectoire.append(next_move)

                dest_reached = True
            # print("Next move found")

            # sinon
            else:
                # si destination non atteinte

                if len(next_wave) == 0:
                    can_propagate = False
                # print("Can't find next move")

                else:
                    prec_wave = next_wave

            wave_number += 1

        return trajectoire

    def WPAUnexplored(self, pos_init,moore=True, destination=None):
        dest_reached = False

        if destination == pos_init:
            # we don't want the wave to run forever
            return []

        total_propag = set()
        prec_wave = set([pos_init])
        dict_paths = {}

        wave_number = 1
        trajectoire = []

        # print("\nDestination Robot {0}: {1}".format(num_robot, destination))

        while not dest_reached:
            next_wave = set()
            total_propag = total_propag.union(prec_wave)

            # récupérer les coordonnées des voisins accessibles
            for cell in prec_wave:
                accessible_neighbors = []
                neighbors = self.determineCellNeighborsCoordinates(cell, moore)
                # print("accessible_neighbors before filtering :" + str(accessible_neighbors))
                for neighbor in neighbors:
                    # On enlève les obstacles connus
                    if neighbor not in self.getWalls():
                        accessible_neighbors.append(neighbor)

                # print("accessible_neighbors after filtering :" + str(accessible_neighbors))

                # ajout de ces voisins à l'ensemble des voisins potentiels de la vague suivante
                local_next_wave = set(accessible_neighbors)
                # print("Next wave potentielle:", sorted(local_next_wave))

                # si vague déjà propagée sur la case, on enlève la case en question
                local_next_wave -= total_propag
                # print("Next wave sans cases déjà propagées:", sorted(local_next_wave))

                # on propage la vague sur les cases restantes accessibles
                for cell_next in local_next_wave:
                    # on regarde si la case suivante a déjà des précédesseurs connus
                    # sinon on créé un ensemble vide
                    set_prec = dict_paths.get(cell_next, set())

                    # on ajoute à cet ensemble vide notre case précédente, ici il s'agit de "cell"
                    set_prec.add(cell)

                    # on stocke dans le dictionnaire à la clé "cell_next" notre ensemble
                    dict_paths[cell_next] = set_prec

                # print("Cellules précédentes de {0}: {1}".format(cell_next, dict_paths[cell_next]))

                # ajout de cette mini-vague à la prochaine vague complète
                next_wave = next_wave.union(local_next_wave)

            # print("Wave number ({1}) for {0}:".format(num_robot, wave_number), sorted(next_wave))

            # si la destination fait partie de ces cases, destination trouvée
            if destination in next_wave:
                next_move = destination
                trajectoire.append(destination)

                # récupérer la case suivante à accéder
                while pos_init not in dict_paths[next_move]:
                    # on prend au hasard une des cases de l'ensemble
                    next_move = dict_paths[next_move].pop()
                    trajectoire.append(next_move)

                dest_reached = True
            # print("Next move found")

            # sinon
            else:
                # si destination non atteinte
                prec_wave = next_wave

            wave_number += 1
            # We keep the while to go on indefinitely
            if len(next_wave) == 0 or wave_number > 150:
                # print("Can't find next move")
                return -1

        return trajectoire

    def findShortestTrajectory(self, pos_init, destinations):
        destination_found = False

        total_propag = set()
        prec_wave = set([pos_init])
        dict_paths = {}

        wave_number = 1
        dest = pos_init

        trajectoire = []

        while (not destination_found):

            next_wave = set()
            total_propag = total_propag.union(prec_wave)

            # récupérer les coordonnées des voisins accessibles
            for cell in prec_wave:
                accessible_neighbors = []
                neighbors = self.determineCellNeighborsCoordinates(cell, None)
                # print("accessible_neighbors before filtering :" + str(accessible_neighbors))
                for neighbor in neighbors:
                    # On enlève les obstacles connus
                    if neighbor not in self.getWalls():
                        accessible_neighbors.append(neighbor)

                # print("accessible_neighbors after filtering :" + str(accessible_neighbors))

                # ajout de ces voisins à l'ensemble des voisins potentiels de la vague suivante
                local_next_wave = set(accessible_neighbors)
                # print("Next wave potentielle:", sorted(local_next_wave))

                # si vague déjà propagée sur la case, on enlève la case en question
                local_next_wave -= total_propag
                # print("Next wave sans cases déjà propagées:", sorted(local_next_wave))

                # on propage la vague sur les cases restantes accessibles
                for cell_next in local_next_wave:
                    # on regarde si la case suivante a déjà des précédesseurs connus
                    # sinon on créé un ensemble vide
                    set_prec = dict_paths.get(cell_next, set())

                    # on ajoute à cet ensemble vide notre case précédente, ici il s'agit de "cell"
                    set_prec.add(cell)

                    # on stocke dans le dictionnaire à la clé "cell_next" notre ensemble
                    dict_paths[cell_next] = set_prec

                # print("Cellules précédentes de {0}: {1}".format(cell_next, dict_paths[cell_next]))

                # ajout de cette mini-vague à la prochaine vague complète
                next_wave = next_wave.union(local_next_wave)

            # on vérifie pour chaque cell de la next wave s'il s'agit d'une frontière ou pas
            for cell in next_wave:
                if cell in destinations:
                    # aka next_move
                    dest = cell
                    next_move = dest
                    destination_found = True

                    # récupérer tout le trajet pour y accéder
                    trajectoire.append(next_move)
                    # récupérer la case suivante à accéder
                    while (pos_init not in dict_paths[next_move]):
                        # on prend au hasard une des cases de l'ensemble
                        next_move = dict_paths[next_move].pop()
                        trajectoire.append(next_move)
                    break

            if len(next_wave) == 0 or wave_number > 150:
                # print("Can't find next move")
                return None, -1

            else:
                prec_wave = next_wave

            wave_number += 1

        return dest, trajectoire

    def WPA_without_robots_interferences(self, pos_init, coord_robots, moore=True,destination=None):
        dest_reached = False
        can_propagate = True

        total_propag = set()
        prec_wave = set([pos_init])
        dict_paths = {}

        wave_number = 1
        next_move = pos_init

        trajectoire = []

        # print("\nDestination Robot {0}: {1}".format(num_robot, destination))

        while (not dest_reached) and can_propagate:

            next_wave = set()
            total_propag = total_propag.union(prec_wave)

            # get the neighbours coordinates
            for cell in prec_wave:
                # cell contending neither walls nor explored areas
                accessible_neighbors = self.getAccessibleNeighbors(cell,moore)

                # print("Voisins accessibles depuis {0}:".format(cell), sorted(accessible_neighbors))

                # add these neighbours to the set of potentials neighbours for the following wavefront
                local_next_wave = set(accessible_neighbors)
                # print("Next wave potentielle:", sorted(local_next_wave))

                # if wave already spread on a cell, we remove it
                local_next_wave -= total_propag
                # print("Next wave sans cases déjà propagées:", sorted(local_next_wave))

                # we spread out the wave to the remain accessible cells
                for cell_next in local_next_wave:
                    # on regarde si la case suivante a déjà des précédesseurs connus
                    # sinon on créé un ensemble vide
                    set_prec = dict_paths.get(cell_next, set())

                    # on ajoute à cet ensemble vide notre case précédente, ici il s'agit de "cell"
                    set_prec.add(cell)

                    # on stocke dans le dictionnaire à la clé "cell_next" notre ensemble
                    dict_paths[cell_next] = set_prec

                # print("Cellules précédentes de {0}: {1}".format(cell_next, dict_paths[cell_next]))

                # ajout de cette mini-vague à la prochaine vague complète
                next_wave = next_wave.union(local_next_wave)

            # print("Wave number ({1}) for {0}:".format(num_robot, wave_number), sorted(next_wave))

            # si la destination fait partie de ces cases, destination trouvée
            if destination in next_wave:
                next_move = destination
                trajectoire.append(destination)

                # récupérer la case suivante à accéder
                while pos_init not in dict_paths[next_move]:
                    # on prend au hasard une des cases de l'ensemble
                    next_move = dict_paths[next_move].pop()
                    trajectoire.append(next_move)

                dest_reached = True
            # print("Next move found")

            # sinon
            else:
                # si destination non atteinte

                if len(next_wave) == 0:
                    can_propagate = False
                # print("Can't find next move")

                else:
                    prec_wave = next_wave

            wave_number += 1

        return trajectoire

    def addCellExplored(self, cell):
        """
        cell is a tuple (x, y)
        """
        self.cells_explored.add(cell)

    def addFrontierToExplore(self, cell):
        self.frontiers_to_explore.add(cell)

    def addCost(self, cell, cost):
        self.costGrid.update({cell: cost})

    def addUtility(self, cell, utility):
        self.utilityGrid.update({cell: utility})

    def addEntropy(self, cell, entropy):
        self.entropyGrid.update({cell: entropy})

    def addOccupancy(self, cell, occupancy):
        self.occupancyGrid.update({cell: occupancy})

    def addTarget(self, cell):
        self.target.add(cell)

    def addWall(self, cell):
        self.walls.add(cell)

    def getAccessibleNeighbors(self, cell, moore=True,coord_robots=None):
        list_neighbors = self.determineCellNeighborsCoordinates(cell, moore)
        final_list = []
        for cell in list_neighbors:
            # si la cellule n'est pas un mur et qu'elle a déjà été explorée
            if ((cell not in self.getWalls()) and (cell in self.getCellsExplored())):
                if coord_robots != None:
                    if (cell not in coord_robots):
                        final_list.append(cell)
                else:
                    final_list.append(cell)
        return final_list

    def determineCellNeighborsCoordinates(self, cell, moore=True):
        # cell is a tuple (x, y)
        with_moore = [(-1, 0), (0, -1), (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]
        to_calculate = [(-1, 0), (0, -1), (0, 1), (1, 0)]

        list_neighbors = []
        x, y = cell

        # calculate with moore
        if moore:
            to_calculate = with_moore

        for (xp, yp) in to_calculate:
            xn, yn = x + xp, y + yp
            if (xn >= 0 and yn >= 0):
                list_neighbors.append((xn, yn))

        return list_neighbors

        # moore determine if we have to include diagonals or not

    def determineFrontiers(self, moore=True):
        for cell in self.getCellsExplored():
            # calculate neighbors coordinates
            list_neighbors = self.determineCellNeighborsCoordinates(cell, moore)

            is_in_frontiers = False
            if cell in self.getFrontiersToExplore():
                is_in_frontiers = True

            is_frontier = False
            # si un des voisins de cette cellule n'a pas été explorée, alors il s'agit d'une frontière
            for cell_neighbor in list_neighbors:
                # la cellule voisine n'est pas explorée donc la cellule actuelle est une frontière
                # si la cellule analysée appartient déjà aux murs alors impossible à exploiter
                if (cell_neighbor not in self.getCellsExplored()) and (cell_neighbor not in self.getWalls()):
                    self.addFrontierToExplore(cell)
                    if cell not in self.utilityGrid:
                        self.addUtility(cell, 1)
                    is_frontier = True

            # si la cellule n'est plus considérable comme frontière mais qu'elle fait toujours partie de l'ensemble frontière, l'enlever
            if not is_frontier and is_in_frontiers:
                self.frontiers_to_explore.remove(cell)
                if cell in self.utilityGrid:
                    self.utilityGrid.pop(cell)

class LocalMap(MapRobot):
    def __init__(self, init_pos):
        super().__init__()
        self.pos_robot = init_pos  # tuple (x, y)
        self.path_followed = []

    def updatePosition(self, new_pos):
        self.path_followed.append(self.pos_robot)
        self.pos_robot = new_pos

    def getPathFollowed(self):
        return self.path_followed

class GlobalMap(MapRobot):
    def __init__(self, n_robots):
        super().__init__()
        self.position_robots = {}
        self.cells_discoverer = {}

        self.createEmptyMatrix(n_robots)

    def createEmptyMatrix(self, n_robots):
        for i in range(n_robots):
            self.cells_discoverer[str(i)] = []

    def updatePositionRobot(self, num_robot, new_pos):
        self.position_robots[str(num_robot)] = new_pos

    def getListPositionsRobots(self):
        return list(self.position_robots.values())

    def getCellsDiscoverer(self):
        return self.cells_discoverer

    def getCellByDiscoverer(self, unique_id):
        return self.cells_discoverer.get(str(unique_id), [(0, 0)])

    def addCellExplored(self, cell, unique_id):
        """
        cell is a tuple (x, y)
        """
        if cell not in self.cells_explored:
            self.cells_explored.add(cell)

            # ajouter celui qui l'a découverte
            self.cells_discoverer[str(unique_id)].append(cell)


class BeliefGrid:
    def __init__(self, width=0, height=0, positions_target=[],positions_fakeTargets=[]):
        self.width = width
        self.height = height
        self.nbTarget= len(positions_target)
        # unreachable cells
        self.unreachableCells = set()
        # probability grid, evolving with each observation
        self.grid = [[0 for a in range(self.width)] for b in range(self.height)]
        # initializing all probabilities in the grid
        for i in range(self.width):
            for j in range(self.height):
                self.grid[i][j] = 0.5
                
        for i in range(self.width):
            for j in range(self.height):
                for h in positions_target:
                    if h[0]==i and h[1]==j:
                        self.grid[i][j] = 0.8
                        self.grid[i+1][j] = 0.75
                        self.grid[i-1][j] = 0.75
                        self.grid[i][j+1] = 0.75
                        self.grid[i][j-1] = 0.75
                        self.grid[i+2][j] = 0.70
                        self.grid[i-2][j] = 0.70
                        self.grid[i][j+2] = 0.70
                        self.grid[i][j-2] = 0.70
                        self.grid[i+1][j-1] = 0.70
                        self.grid[i-1][j-1] = 0.70
                        self.grid[i+1][j+1] = 0.70
                        self.grid[i-1][j+1] = 0.70
        for i in range(self.width):
            for j in range(self.height):
                for h in positions_fakeTargets:
                    if h[0]==i and h[1]==j:
                        self.grid[i][j] = 0.8
                        self.grid[i+1][j] = 0.75
                        self.grid[i-1][j] = 0.75
                        self.grid[i][j+1] = 0.75
                        self.grid[i][j-1] = 0.75
                # self.grid[i][j] = random.randint(3, 7)/10
        
        
    def isPotentialTarget(self,i,j):
        if self.grid[i][j]>=0.6:
            return True,self.grid[i][j]/10
        return False,self.grid[i][j]/10

    # self.presenceProbability / (self.model.width * self.model.height)
    # print("i,j = " + str(i) + "," + str(j) + " | " + str(self.beliefGrid[i][j]))

    def update(self, visible_cell, observation):
        xC, yC = visible_cell
        previousCellBelief = self.grid[xC][yC]
        for i in range(self.width):
            for j in range(self.height):
                if (i, j) == (xC, yC):
                    if observation == 0:
                        self.grid[i][j] = (observation + previousCellBelief) / 2
                    if observation != 0:
                        self.grid[i][j]=0
                # else:
                    # if observation == 0:
                    #     self.grid[i][j] = (self.grid[i][j] +
                    #                        (self.grid[i][j] + (
                    #                                ((observation + previousCellBelief) / 2) / (
                    #                                self.width * self.height)))) / 2
                #     if observation!= 0:
                #         self.grid[i][j] = (self.grid[i][j] +
                #                            (self.grid[i][j] - (
                #                                    ((observation + previousCellBelief) / 2) / (
                #                                    self.width * self.height)))) / 2
                if self.grid[i][j] > 1:
                    self.grid[i][j] = 1
                

        for unreachable_cell in self.unreachableCells:
            xU, yU = unreachable_cell
            self.grid[xU][yU] = 0
