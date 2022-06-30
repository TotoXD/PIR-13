import json
import glob
import csv
def read_grid_txt(filename):
	# get the walls
	list_walls = []
	list_robots = []

	# open the file
	with open(filename, 'r') as file:
		# for each line and each character in it
		x = 0
		for line in file:
			y = 0
			for char in line:
				# if it's identified as wall, add it
				if char == '1':
					list_walls.append((x,y))
				elif char == '2':
					list_robots.append((x,y))
				y += 1
			x += 1

	return list_walls, x, y, list_robots

def generate_wall(x_begin, y_begin, x_end, y_end):
	""" Create the wall by generating the coordinates between the beginning and the end """
	walls = []

	# we create a list to finally return a set (of unique coordinates) to avoid overlap and drawing problems

	if (x_begin - x_end == 0) and (y_begin - y_end == 0):
		# only one cell
		walls = set([(x_begin, y_begin)])
	elif x_begin - x_end == 0:
		# draw one line vertically
		start, end = min(y_begin, y_end), max(y_begin, y_end)
		walls = set([(x_begin, j) for j in range(start, end+1, 1)])
	elif y_begin - y_end == 0:
		# draw one line horizontally
		start, end = min(x_begin, x_end), max(x_begin, x_end)
		walls = set([(i, y_begin) for i in range(start, end+1, 1)])
	else:
		# draw a rectangle
		x_start, x_end = min(x_begin, x_end), max(x_begin, x_end)
		y_start, y_end = min(y_begin, y_end), max(y_begin, y_end)
		walls = set([(i, j) for i in range(x_start, x_end+1) for j in range(y_start, y_end+1)])

	return walls

import sys
from os import path


def verification_parameters():
	# ignores 'python3'
	# 1st : run.py
	# 2nd : filename

	set_walls = []
	positions_robots = None
	positions_targets = None

	HEIGHT = 0
	WIDTH = 0

	if len(sys.argv) != 2:
		print("Incorrect number of arguments")
		sys.exit(-1)
	else:
		path_file = sys.argv[1]
		_, extension = path_file.split('.')
		print("Extension:", extension)

		if extension in ['json', 'csv', 'txt']:
			# valid extension

			# if the file exists
			if (path.exists(path_file)):
				# now read the file
				if (extension == "json"):
					set_walls, HEIGHT, WIDTH, positions_robots = read_grid_json(path_file)
				elif (extension == "txt"):
					set_walls, HEIGHT, WIDTH, positions_robots = read_grid_txt(path_file)
				elif (extension == "csv"):
					set_walls, HEIGHT, WIDTH, positions_robots, positions_targets, position_fakeTargets = read_grid_csv(path_file)
			else:
				print("File does not exists at that path:", path_file)
				sys.exit(-1)
		else:
			print("Invalid extension:", extension)
			sys.exit(-1)
	return set_walls, HEIGHT, WIDTH, path_file, positions_robots, positions_targets, position_fakeTargets

def read_grid_json(filename):
	# Opening JSON file
	f = open(filename,)

	# returns JSON object as
	# a dictionary
	data = json.load(f)

	height = data['height']
	width = data['width']
	list_walls = []
	list_robots = []

	for wall in data['walls']:
		# identify if it's an horizontal wall or a vertical one
		x_begin, y_begin = wall["begin"]
		x_end, y_end = wall["end"]

		new_walls = generate_wall(x_begin, y_begin, x_end, y_end)

		list_walls += new_walls


	if 'robots' in data:
		for robot in data['robots']:
			x, y = robot["position"]

			list_robots.append((x,y))

	f.close()
	return list_walls, height, width, list_robots

def read_grid_csv(filename):
	with open(filename, newline='') as csvfile:
		reader = list(csv.reader(csvfile, delimiter=','))
		height = len(reader)
		width = len(reader[0])
		list_walls = []
		list_robots = []
		list_targets = []
		list_fakeTargets=[]

		for y in range(height):
			for x in range(width):
				if reader[height-1-y][x] == '1':
					list_walls.append((x, y))
				elif reader[height-1-y][x] == '2':
					print("Robot at position:({0},{1})".format(x, y))
					# cela signifie qu'il y a un robot à placer
					list_robots.append((x, y))
				elif reader[height-1-y][x] == '3':
					print("Target at position:({0},{1})".format(x, y))
					# cela signifie qu'il y a un robot à placer
					list_targets.append((x, y))
				elif reader[height-1-y][x] == '4':
					print("FakeTarget at position:({0},{1})".format(x, y))
					# cela signifie qu'il y a un robot à placer
					list_fakeTargets.append((x, y))

	return list_walls, height, width, list_robots, list_targets, list_fakeTargets