https://drive.google.com/drive/u/1/folders/113e0yslAq7l-ULSH0pAOAE9a4xl3vOg8

## Pour lancer le simulateur
1. démarrer un terminal et se placer dans le dossier simulation
2. saisir la commande *python3 run.py <chemin_de_la_carte>*
3. ouvrir un navigateur et se rendre à l'addresse [http://127.0.0.1:8521/](http://127.0.0.1:8521/)

## Programme

### ``run_batches.py``

- utilisé pour effectuer un grand nombre de simulations rapidement

- Réglage des paramètres :
  - aller dans le fichier run_batches.py
  - modifier les paramètres fixed_params et variable_params en fonction des besoins
  - il s'agit des paramètres fournis au modèle, attention donc aux noms des clés dans le dictionnaire. 

- Pour le lancer :
  1. se mettre dans le dossier *simulation*
  2. saisir la commande *python3 run_batches.py <chemin_de_la_carte>*
- Le programme s'arrêtera quand toutes les itérations auront été effectuées
- Les résultats seront disponibles dans le dossier "résultats"

### ``analysis.py``

- Utiliser pour pouvoir analyser les résultats présents dans results.csv
- Il s'agit d'un programme de filtre et tri qui affiche ensuite les résultats sous forme de tableau et graphe

ATTENTION Bien vérifier que les résultats que l'on veut afficher se trouve bien dans results.csv sinon il y aura une erreur ATTENTION

- Pour le lancer :
  1. Se mettre dans le dossier *simulation*
  2. Saisir la commande *python3 .\analysis.py map_name number_robots radius_robots bRadius beta type_robots* en choisissant un des paramètres (number_robots, radius_robots, bRadius, beta) qui sera alors la variable tandis que les autres seront fixes dans les graphes, pour ce faire le paramètre variable doit être écrit en toute lettre. L'argument *type_robots* permet de choisir quelle(s) stratégie(s) afficher pour les comparaisons.

Exemple : *python3 .\analysis.py jk_20x20 number_robots 3 3 1 Burgard,Yamauchi*

number_robots est la variable avec laquelle on veut faire notre comparaison avec map_name, radius_robots, bRadius, beta qui restent fixes. On a choisi Burgard et Yamauchi comme type de robots pour les comparer.
