from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from werkzeug.utils import secure_filename
from flask import Flask, render_template, request, session

import math
import subprocess
import os
import sys
#import serial
import json
import time
import csv

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
	if request.method == 'GET':
		content = request.json
		sortlist = json.dumps(content)
		insorted = [sortlist]
		data = {}
		unsorted = [
        (288, 149), (288, 129), (270, 133), (256, 141), (256, 157), (246, 157),
        (236, 169), (228, 169), (228, 161), (220, 169), (212, 169), (204, 169),
        (196, 169), (188, 169), (196, 161), (188, 145), (172, 145), (164, 145),
        (156, 145), (148, 145), (140, 145), (148, 169), (164, 169), (172, 169),
      	(228, 85), (228, 93), (236, 93), (236, 101), (228, 101), (228, 109),
        (228, 117), (228, 125), (220, 125), (212, 117), (204, 109), (196, 101),
        (188, 93), (180, 93), (180, 101), (180, 109), (180, 117), (180, 125),
        (196, 145), (204, 145), (212, 145), (220, 145), (228, 145), (236, 145),
        (246, 141), (252, 125), (260, 129), (280, 133)
    	]  
		print(type(insorted))
		print(insorted)
		print(type(unsorted))
		print(unsorted)
		

		# Create the routing index manager.
		#data['locations'] = [unsorted]
		data['locations'] = unsorted
		data['num_vehicles'] = 1
		data['depot'] = 0
		manager = pywrapcp.RoutingIndexManager(len(data['locations']), data['num_vehicles'], data['depot'])
			
		# Create Routing Model.
		routing = pywrapcp.RoutingModel(manager)

		distance_matrix = compute_euclidean_distance_matrix(data['locations'])

		def distance_callback(from_index, to_index):
			"""Returns the distance between the two nodes."""
			# Convert from routing variable Index to distance matrix NodeIndex.
			from_node = manager.IndexToNode(from_index)
			to_node = manager.IndexToNode(to_index)
			return distance_matrix[from_node][to_node]

		transit_callback_index = routing.RegisterTransitCallback(distance_callback)

		# Define cost of each arc.
		routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

		# Setting first solution heuristic.
		search_parameters = pywrapcp.DefaultRoutingSearchParameters()
		search_parameters.first_solution_strategy = (
			routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

		# Solve the problem and time the process
		start_time = time.time()
		solution = routing.SolveWithParameters(search_parameters)
		elapse = time.time() - start_time
		print("TSP Complete")

		# Create list of ordered pairs and print to txt file
		if solution:
			ordered_pairs = ordered_solution(manager, routing, solution, unsorted)
			#print_solution(ordered_pairs, 'static/' + tspfile)
			#print_to_port('static/' + tspfile)
			#return render_template('complete.html', elapse_time = elapse)
			
			#print(ordered_pairs)
			return json.dumps(ordered_pairs)

def compute_euclidean_distance_matrix(locations):
	"""Creates callback to return distance between points."""
	distances = {}
	for from_counter, from_node in enumerate(locations):
		distances[from_counter] = {}
		for to_counter, to_node in enumerate(locations):
			if from_counter == to_counter:
				distances[from_counter][to_counter] = 0
			else:
				# Euclidean distance
				distances[from_counter][to_counter] = (int(
					math.hypot((from_node[0] - to_node[0]),
							   (from_node[1] - to_node[1]))))
	return distances

def ordered_solution(manager, routing, solution, coordinates):
	ordered_pairs = [] 
	index = routing.Start(0)
	while not routing.IsEnd(index):
		ordered_pairs.append(coordinates[manager.IndexToNode(index)])
		index = solution.Value(routing.NextVar(index))
	return ordered_pairs

if __name__=='__main__':
	app.run(debug=True, port=5000, host='0.0.0.0')
