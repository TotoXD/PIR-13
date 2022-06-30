import sys
from numpy import number, var
import pandas as pd
from pyparsing import col
from tabulate import tabulate
import matplotlib as pt
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import matplotlib.patches as mpatches



pd.set_option("display.max_rows", None, "display.max_columns", None)

df = pd.read_csv("results/results.csv")

# moyenne pour même carte, même nombre de robot, même radius et même stratégie
def getMeanForSameParameters(data, parameters, metrics, metrics_std, map_name,number_robots,radius_robots,bRadius,beta,type_robots):
	# perform the gorup by
	df_grouped = data.groupby(parameters)

	df_mean = df_grouped.agg(metrics)
	df_mean = df_mean.reset_index()

	df_std = df_grouped.agg(metrics_std)
	df_std = df_std.reset_index()

	# Tableau trié selon la map / Tableau d'erreurs trié selon la map
	df_specified_map = df_mean[df_mean["map_name"] == map_name]
	df_specified_map_std = df_std[df_std["map_name"] == map_name]

	# Print tableau selon la carte en argument
	print("\n")
	print("********* RESULTATS POUR LA CARTE DEMANDEE *********")
	print(tabulate(df_specified_map, headers=list(df_specified_map.columns),showindex=False,tablefmt='psql'))
	print("\n")

	# Print tab erreurs
	print("\n")
	print("********* ECART-TYPE SUR LES PRECEDENTES VALEURES *********")
	print(tabulate(df_specified_map_std, headers=list(df_specified_map.columns),showindex=False,tablefmt='psql'))
	print("\n")


	# Affichage des graphes par rapport à la map donnee en argument
	plot(df_specified_map,df_specified_map_std,number_robots,radius_robots,bRadius,beta,type_robots)

def analysis(data, parameters, metrics, metrics_std, map_name="map_25x25",number_robots="number_robots",radius_robots="radius_robots",bRadius="bRadius",beta="beta",type_robots="all"):
	# print("Dataframe initial:\n", data)
	getMeanForSameParameters(data, parameters, metrics, metrics_std, map_name,number_robots,radius_robots,bRadius,beta,type_robots)

def plot(df_specified_map,df_specified_map_std,number_robots,radius_robots,bRadius,beta,type_robots):
	
	# fig, ax = plt.subplots()

	# Les couleurs qui seront utilisees comme legende (il en faut autant que le nombre maximum de strategies comparables possible en meme temps)
	colours = ['#96C5F7','#FF5A5F','#49DCB1','#82204A','#E3B505','#02182B']

	# ****************** MISE EN PLACE DES FILTRES PAR RAPPORT AUX TYPES CHOISIS ****************** #

	if type_robots == "all":

		# On ne change rien
		df_specified_type_robots = df_specified_map
		df_errors_specified_type_robots = df_specified_map_std

	else :
		list_type_robots = type_robots.split(",")

		# On ne garde que les types choisis
		df_specified_type_robots = df_specified_map[df_specified_map['type_robots'].isin(list_type_robots)]
		df_errors_specified_type_robots = df_specified_map_std[df_specified_map_std['type_robots'].isin(list_type_robots)]


	# ****************** MISE EN PLACE DES FILTRES PAR RAPPORT A LA VARIABLE CHOISIE ****************** #

	# Si le paramètre variable est le nombre de robots
	if number_robots == "number_robots":

		# On tri le grand tableau pour garder que ce que l'on veut
		df_only_bRadius = df_specified_type_robots[df_specified_type_robots["bRadius"] == int(bRadius)]
		df_only_bRadius_and_radius = df_only_bRadius[df_only_bRadius["radius_robots"] == int(radius_robots)]
		df_only_bRadius_and_radius_and_beta = df_only_bRadius_and_radius[df_only_bRadius_and_radius["beta"] == float(beta)]

		df_plot = df_specified_map[df_specified_map["map_name"]=="Nothing"]
		for i in range (df_only_bRadius_and_radius_and_beta["number_robots"].max()):
			df_plot = pd.concat([df_only_bRadius_and_radius_and_beta[df_only_bRadius_and_radius_and_beta["number_robots"]==i+1],df_plot])
		df_plot = df_plot.iloc[::-1]

		# Pareil pour le tableau des erreurs
		df_errors_bRadius = df_errors_specified_type_robots[df_errors_specified_type_robots["bRadius"] == int(bRadius)]
		df_errors_bRadius_and_radius = df_errors_bRadius[df_errors_bRadius["radius_robots"] == int(radius_robots)]
		df_errors_bRadius_and_radius_and_beta = df_errors_bRadius_and_radius[df_errors_bRadius_and_radius["beta"] == float(beta)]

		df_errors_plot = df_errors_specified_type_robots[df_errors_specified_type_robots["map_name"]=="Nothing"]
		for i in range (df_errors_bRadius_and_radius_and_beta["number_robots"].max()):
			df_errors_plot = pd.concat([df_errors_bRadius_and_radius_and_beta[df_errors_bRadius_and_radius_and_beta["number_robots"]==i+1],df_errors_plot])
		df_errors_plot = df_errors_plot.iloc[::-1]

		# df_errors_only_Burgard = df_errors_bRadius_and_radius_and_beta[df_errors_bRadius_and_radius_and_beta["type_robots"]=="Burgard"]
		# df_errors_only_Yamauchi = df_errors_bRadius_and_radius_and_beta[df_errors_bRadius_and_radius_and_beta["type_robots"]=="Yamauchi"]
		# df_errors_only_MinPos = df_errors_bRadius_and_radius_and_beta[df_errors_bRadius_and_radius_and_beta["type_robots"]=="MinPos"]
		# df_errors_only_RandomAnt = df_errors_bRadius_and_radius_and_beta[df_errors_bRadius_and_radius_and_beta["type_robots"]=="RandomAnt"]


		# On prend les valeurs des erreurs pour pouvoir les afficher (besoin d'une liste et non d'un dataframe)
		errors_num_steps = df_errors_plot['num_steps'].values.tolist()
		errors_efficency = df_errors_plot['efficiency'].values.tolist()
		errors_calcul_cost = df_errors_plot['calcul_time'].values.tolist()

		# errors_num_steps_only_Burgard = df_errors_only_Burgard['num_steps'].values.tolist()
		# errors_efficency_only_Burgard = df_errors_only_Burgard['efficiency'].values.tolist()

		# errors_num_steps_only_Yamauchi = df_errors_only_Yamauchi['num_steps'].values.tolist()
		# errors_efficency_only_Yamauchi = df_errors_only_Yamauchi['efficiency'].values.tolist()

		# errors_num_steps_only_MinPos = df_errors_only_MinPos['num_steps'].values.tolist()
		# errors_efficency_only_MinPos = df_errors_only_MinPos['efficiency'].values.tolist()

		# errors_num_steps_only_RandomAnt = df_errors_only_RandomAnt['num_steps'].values.tolist()
		# errors_efficency_only_RandomAnt = df_errors_only_RandomAnt['efficiency'].values.tolist()

		# number_robots_value = df_only_bRadius_and_radius['number_robots'].values.tolist()
		# num_steps_value = df_only_bRadius_and_radius['num_steps'].values.tolist()
		# efficiency_value = df_only_bRadius_and_radius['efficiency'].values.tolist()

		# print(num_steps_value)

		# On print le tableau trié
		print("\n")
		print(tabulate(df_plot, headers='keys',showindex=False,tablefmt='psql'))
		print("\n")

		# On cree le dict qui va nous permettre de print les couleurs correspondantes aux differents strategies		
		legend_dict= dict(zip(list_type_robots,colours))

		patchList = []
		for key in legend_dict:
				data_key = mpatches.Patch(color=legend_dict[key], label=key)
				patchList.append(data_key)

		# plot1Burgard = df_only_Burgard.plot(x=number_robots, y='num_steps', yerr=errors_num_steps_only_Burgard, kind="bar", title="portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_only_bRadius_and_radius_and_beta['type_robots'].replace(colours))
		# plot1Yamauchi = df_only_Yamauchi.plot(ax = plot1Burgard, x=number_robots, y='num_steps', yerr=errors_num_steps_only_Yamauchi, kind="bar", title="portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_only_bRadius_and_radius_and_beta['type_robots'].replace(colours))
		# plot1MinPos = df_only_MinPos.plot(ax = plot1Yamauchi, x=number_robots, y='num_steps', yerr=errors_num_steps_only_MinPos, kind="bar", title="portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_only_bRadius_and_radius_and_beta['type_robots'].replace(colours))
		# plot1RandomAnt = df_only_RandomAnt.plot(ax = plot1MinPos, x=number_robots, y='num_steps', yerr=errors_num_steps_only_RandomAnt, kind="bar", title="portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_only_bRadius_and_radius_and_beta['type_robots'].replace(colours))

		# On crée le premier graphe pour visualiser les num_steps
		# firstPlot = df_plot.plot(x=number_robots, y='num_steps', yerr=errors_num_steps, kind="bar",title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(colours))

		firstPlot = df_plot.plot(x=number_robots, y='num_steps', yerr=errors_num_steps, kind="bar",title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		firstPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		# firstPlot.legend([Patch(facecolor=colours['Burgard']), Patch(facecolor=colours['Yamauchi']), Patch(facecolor=colours['RandomAnt']), Patch(facecolor=colours['MinPos']), Patch(facecolor=colours['MinPos2'])], list_type_robots)
		#firstPlot.xticks(number_robots,'num_steps',rotation='horizontal')
		firstPlot.set_xlabel('Nombre de robots')
		firstPlot.set_ylabel('Nombre d\'iterations')
		#firstPlot.errorbar(number_robots,'num_steps', yerr=errors_num_steps, fmt='o', linewidth=2, capsize=6)

		# On crée le second graphe pour visualiser l'efficiency
		secondPlot = df_plot.plot(x= number_robots, y='efficiency', yerr=errors_efficency, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		secondPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		secondPlot.set_xlabel('Nombre de robots')
		secondPlot.set_ylabel('Rendement')

		# On crée le troisieme graphe pour visualiser l'efficiency
		thirdPlot = df_plot.plot(x= number_robots, y='calcul_time', yerr=errors_calcul_cost, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"portée = "+radius_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		thirdPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		thirdPlot.set_xlabel('Nombre de robots')
		thirdPlot.set_ylabel('Temps en calcul')

	# Si le paramètre variable est la portee des robots
	elif radius_robots == "radius_robots":

		# On tri le grand tableau pour garder que ce que l'on veut
		df_only_number_robots = df_specified_type_robots[df_specified_type_robots["number_robots"] == int(number_robots)]
		df_only_number_and_bRadius = df_only_number_robots[df_only_number_robots["bRadius"] == int(bRadius)]
		df_only_number_and_bRadius_and_beta = df_only_number_and_bRadius[df_only_number_and_bRadius["beta"] == float(beta)]

		df_plot = df_specified_map[df_specified_map["map_name"]=="Nothing"]
		for i in range (df_only_number_and_bRadius_and_beta["radius_robots"].max()):
			df_plot = pd.concat([df_only_number_and_bRadius_and_beta[df_only_number_and_bRadius_and_beta["radius_robots"]==i+1],df_plot])
			df_plot = pd.concat([df_only_number_and_bRadius_and_beta[df_only_number_and_bRadius_and_beta["map_name"]=="Nothing"],df_plot])
		df_plot = df_plot.iloc[::-1]

		# Pareil pour le tableau des erreurs
		df_errors_number_robots = df_errors_specified_type_robots[df_errors_specified_type_robots["number_robots"] == int(number_robots)]
		df_errors_number_robots_and_bRadius = df_errors_number_robots[df_errors_number_robots["bRadius"] == int(bRadius)]
		df_errors_number_robots_and_bRadius_and_beta = df_errors_number_robots_and_bRadius[df_errors_number_robots_and_bRadius["beta"] == float(beta)]

		df_errors_plot = df_errors_specified_type_robots[df_errors_specified_type_robots["map_name"]=="Nothing"]
		for i in range (df_errors_number_robots_and_bRadius_and_beta["radius_robots"].max()):
			df_errors_plot = pd.concat([df_errors_number_robots_and_bRadius_and_beta[df_errors_number_robots_and_bRadius_and_beta["radius_robots"]==i+1],df_errors_plot])
		df_errors_plot = df_errors_plot.iloc[::-1]

		# print(df_errors_bRadius_and_radius)

		# On prend les valeurs des erreurs pour pouvoir les afficher (besoin d'une liste et non d'un dataframe)
		errors_num_steps = df_errors_plot['num_steps'].values.tolist()
		errors_efficency = df_errors_plot['efficiency'].values.tolist()
		errors_calcul_cost = df_errors_plot['calcul_time'].values.tolist()

		# On print le tableau trié
		print("\n")
		print(tabulate(df_plot, headers='keys',showindex=False,tablefmt='psql'))
		print("\n")

		# On cree le dict qui va nous permettre de print les couleurs correspondantes aux differents strategies		
		legend_dict= dict(zip(list_type_robots,colours))

		patchList = []
		for key in legend_dict:
				data_key = mpatches.Patch(color=legend_dict[key], label=key)
				patchList.append(data_key)

		# On crée le premier graphe pour visualiser les num_steps
		firstPlot = df_plot.plot(x= radius_robots, y='num_steps', yerr=errors_num_steps, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		firstPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		firstPlot.set_xlabel('Portee des robots')
		firstPlot.set_ylabel('Nombre d\'iterations')
		
		# On crée le second graphe pour visualiser l'efficiency
		secondPlot = df_plot.plot(x= radius_robots, y='efficiency', kind="bar", yerr=errors_efficency, title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et bRadius = "+bRadius+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		secondPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		secondPlot.set_xlabel('Portee des robots')
		secondPlot.set_ylabel('Rendement')

		# On crée le troisieme graphe pour visualiser le cout en calcul
		thirdPlot = df_plot.plot(x= radius_robots, y='calcul_time', yerr=errors_calcul_cost, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		thirdPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		thirdPlot.set_xlabel('Portee des robots')
		thirdPlot.set_ylabel('Temps de calcul')

	# Si le paramètre variable est le bRadius
	elif bRadius == "bRadius":

		# On tri le grand tableau pour garder que ce que l'on veut
		df_only_number_robots = df_specified_type_robots[df_specified_type_robots["number_robots"] == int(number_robots)]
		df_only_number_and_radius = df_only_number_robots[df_only_number_robots["radius_robots"] == int(radius_robots)]
		df_only_number_and_radius_and_beta = df_only_number_and_radius[df_only_number_and_radius["beta"] == float(beta)]

		df_plot = df_specified_map[df_specified_map["map_name"]=="Nothing"]
		for i in range (df_only_number_and_radius_and_beta["bRadius"].max()):
			df_plot = pd.concat([df_only_number_and_radius_and_beta[df_only_number_and_radius_and_beta["bRadius"]==i+1],df_plot])
		df_plot = df_plot.iloc[::-1]

		# Pareil pour le tableau des erreurs
		df_errors_number = df_errors_specified_type_robots[df_errors_specified_type_robots["number_robots"] == int(number_robots)]
		df_errors_number_and_radius = df_errors_number[df_errors_number["radius_robots"] == int(radius_robots)]
		df_errors_number_and_radius_and_beta = df_errors_number_and_radius[df_errors_number_and_radius["beta"] == float(beta)]

		df_errors_plot = df_errors_specified_type_robots[df_errors_specified_type_robots["map_name"]=="Nothing"]
		for i in range (df_errors_number_and_radius_and_beta["bRadius"].max()):
			df_errors_plot = pd.concat([df_errors_number_and_radius_and_beta[df_errors_number_and_radius_and_beta["bRadius"]==i+1],df_errors_plot])
		df_errors_plot = df_errors_plot.iloc[::-1]

		# On prend les valeurs des erreurs pour pouvoir les afficher (besoin d'une liste et non d'un dataframe)
		errors_num_steps = df_errors_plot['num_steps'].values.tolist()
		errors_efficency = df_errors_plot['efficiency'].values.tolist()
		errors_calcul_cost = df_errors_plot['calcul_time'].values.tolist()

		# On print le tableau trié
		print("\n")
		print(tabulate(df_only_number_and_radius_and_beta, headers='keys',showindex=False,tablefmt='psql'))
		print("\n")

		# On cree le dict qui va nous permettre de print les couleurs correspondantes aux differents strategies		
		legend_dict= dict(zip(list_type_robots,colours))

		patchList = []
		for key in legend_dict:
				data_key = mpatches.Patch(color=legend_dict[key], label=key)
				patchList.append(data_key)

		# On crée le premier graphe pour visualiser les num_steps
		firstPlot = df_plot.plot(x= bRadius, y='num_steps', yerr=errors_num_steps, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		firstPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		firstPlot.set_xlabel('bRadius')
		firstPlot.set_ylabel('Nombre d\'iterations')

		# On crée le second graphe pour visualiser l'efficiency
		secondPlot = df_plot.plot(x= bRadius, y='efficiency', yerr=errors_efficency, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		secondPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		secondPlot.set_xlabel('bRadius')
		secondPlot.set_ylabel('Efficiency')

		# On crée le troisième graphe pour visualiser le calcul cost
		thirdPlot = df_plot.plot(x= bRadius, y='calcul_time', yerr=errors_calcul_cost, kind="bar", title="Carte : "+str(df_plot['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et beta = "+beta, color=df_plot['type_robots'].replace(legend_dict))
		thirdPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		thirdPlot.set_xlabel('bRadius')
		thirdPlot.set_ylabel('Temps de calcul')

	elif beta == "beta":

		# On tri le grand tableau pour garder que ce que l'on veut
		df_only_number_robots = df_specified_type_robots[df_specified_type_robots["number_robots"] == int(number_robots)]
		df_only_number_and_radius = df_only_number_robots[df_only_number_robots["radius_robots"] == int(radius_robots)]
		df_only_number_and_radius_and_bRadius = df_only_number_and_radius[df_only_number_and_radius["bRadius"] == int(bRadius)]

		# Pareil pour le tableau des erreurs
		df_errors_number = df_errors_specified_type_robots[df_errors_specified_type_robots["number_robots"] == int(number_robots)]
		df_errors_number_and_radius = df_errors_number[df_errors_number["radius_robots"] == int(radius_robots)]
		df_errors_number_and_radius_and_bRadius = df_errors_number_and_radius[df_errors_number_and_radius["bRadius"] == int(bRadius)]

		# On prend les valeurs des erreurs pour pouvoir les afficher (besoin d'une liste et non d'un dataframe)
		errors_num_steps = df_errors_number_and_radius_and_bRadius['num_steps'].values.tolist()
		errors_efficency = df_errors_number_and_radius_and_bRadius['efficiency'].values.tolist()
		errors_calcul_cost = df_errors_number_and_radius_and_bRadius['calcul_time'].values.tolist()

		# On print le tableau trié
		print("\n")
		print(tabulate(df_only_number_and_radius_and_bRadius, headers='keys',showindex=False,tablefmt='psql'))
		print("\n")

		# On cree le dict qui va nous permettre de print les couleurs correspondantes aux differents strategies		
		legend_dict= dict(zip(list_type_robots,colours))

		patchList = []
		for key in legend_dict:
				data_key = mpatches.Patch(color=legend_dict[key], label=key)
				patchList.append(data_key)

		# On crée le premier graphe pour visualiser les num_steps
		firstPlot = df_only_number_and_radius_and_bRadius.plot(x= beta, y='num_steps', yerr=errors_num_steps, kind="bar", title="Carte : "+str(df_only_number_and_radius_and_bRadius['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et bRadius = "+bRadius, color=df_only_number_and_radius_and_bRadius['type_robots'].replace(legend_dict))
		firstPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		firstPlot.set_xlabel('beta')
		firstPlot.set_ylabel('Nombre d\'iterations')

		# On crée le second graphe pour visualiser l'efficiency
		secondPlot = df_only_number_and_radius_and_bRadius.plot(x= beta, y='efficiency', yerr=errors_efficency, kind="bar", title="Carte : "+str(df_only_number_and_radius_and_bRadius['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et bRadius = "+bRadius, color=df_only_number_and_radius_and_bRadius['type_robots'].replace(legend_dict))
		secondPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		secondPlot.set_xlabel('beta')
		secondPlot.set_ylabel('Efficiency')

		# On crée le troisième graphe pour visualiser le calcul cost
		thirdPlot = df_only_number_and_radius_and_bRadius.plot(x= beta, y='calcul_time', yerr=errors_calcul_cost, kind="bar", title="Carte : "+str(df_only_number_and_radius_and_bRadius['map_name'].iloc[0])+"\n"+"nombre de robots = "+number_robots+" et portée = "+radius_robots+" et bRadius = "+bRadius, color=df_only_number_and_radius_and_bRadius['type_robots'].replace(legend_dict))
		thirdPlot.legend(handles=patchList,ncol=len(list_type_robots), fontsize='small')
		thirdPlot.set_xlabel('beta')
		thirdPlot.set_ylabel('Temps de calcul')

	# On affiche les graphes
	plt.show()

# Les paramètres dont le nombre est certain
parameters = ["map_name", "type_robots", "number_robots", "radius_robots", "bRadius", "beta"]

# Les metriques moyennées qui seront la base des comparaisons
metrics_dict = {"num_steps": 'mean', "efficiency": 'mean', "total_cost":'mean', "exploration_cost": 'mean', "rotation_cost":'mean', "calcul_time":'mean'}

# Les metriques prisent en compte pour les ecarts-type avec une standard deviation
metrics_dict_std = {"num_steps": 'std', "efficiency": 'std', "total_cost":'std', "exploration_cost": 'std', "rotation_cost":'std', "calcul_time":'std'}

# La commmande qui prend les arguments et déclenche les analyses
if len(sys.argv) == 7:
	analysis(df, parameters, metrics_dict, metrics_dict_std, sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	# python3 .\analysis.py map number_robots radius_robots bRadius beta type_robots
else:
	print("Erreur d'arguments")