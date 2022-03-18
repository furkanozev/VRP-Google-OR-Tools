from flask import Flask, request
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = Flask(__name__)

def create_data_model(inputs):
	# Create data model and stores values.
	data = dict()

	# Duration matrix.
	data['duration_matrix'] = inputs['matrix']
	length = len(data['duration_matrix'])
	dummy_depot = [0] * length
	data['duration_matrix'].append(dummy_depot)
	for i in data['duration_matrix']:
		i.append(0)

	# Depot is not used but it may use for other constraints.
	data['depot'] = length
	length += 1


	# Set location indexes and amount of carboy for each job.
	data['demands'] = [0] * length
	data['indexes'] = dict()
	for job in inputs['jobs']:
		location_index = job['location_index']
		job_id = job['id']
		delivery_amount = job['delivery'][0]
		data['demands'][location_index] = delivery_amount
		data['indexes'][location_index] = job_id

	# Set capacity of car and start points.
	data['vehicle_capacities'] = list()
	data['starts'] = list()
	data['vehicle_id'] = list()
	for vehicle in inputs['vehicles']:
		capacity = vehicle['capacity'][0]
		start_index = vehicle['start_index']
		vehicle_id = vehicle['id']
		data['vehicle_capacities'].append(capacity)
		data['starts'].append(start_index)
		data['vehicle_id'].append(vehicle_id)

	# Vehicles dont need to return depot so, i will create dummy depot point,
	# and duration time of this depot is 0 from anywhere.
	data['ends'] = [data['depot']] * len(data['vehicle_capacities'])

	# Set amount of vehicles
	data['num_vehicles'] = len(data['vehicle_capacities'])

	return data

# Convert solution to json format.
def prepare_solution(data, manager, routing, solution):
	res = {'total_delivery_duration': 0, 'routes': {}}

	total_time = 0
	for vehicle_id in range(data['num_vehicles']):
		no_order = 0
		value, previous_value = 0, 0
		flag = False
		index = routing.Start(vehicle_id)
		key = str(data['vehicle_id'][vehicle_id])
		res['routes'][key] = {'jobs': [], 'delivery_duration': 0}
		route_time = 0
		while not routing.IsEnd(index):
			node_index = manager.IndexToNode(index)
			if node_index in data['indexes']:
				res['routes'][key]['jobs'].append(data['indexes'][index])
				flag = True
				no_order = 0
			else:
				flag = False
			previous_index = index
			index = solution.Value(routing.NextVar(index))
			value = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
			route_time += value
			if flag == False:
				no_order += previous_value
			previous_value = value
		if flag == False:
			route_time -= no_order

		total_time += route_time
		res['routes'][key]['delivery_duration'] = route_time

	res['total_delivery_duration'] = total_time
	
	return res

# getRoute RestAPI point, it takes an json then return route response, if it exists.
@app.route('/getRoute', methods=['POST'])
def get_route():
	inputs = request.json

	if inputs is None:
		return {'result': False}, 400

	# Input data model.
	data = create_data_model(inputs)

	# Create routing index manager.
	manager = pywrapcp.RoutingIndexManager(len(data['duration_matrix']), data['num_vehicles'], data['starts'], data['ends'])

	# Create routing model.
	routing = pywrapcp.RoutingModel(manager)

	# Create and register a transit callback.
	# Return duration time between given 2 index.
	def time_callback(from_index, to_index):
		from_node = manager.IndexToNode(from_index)
		to_node = manager.IndexToNode(to_index)
		return data['duration_matrix'][from_node][to_node]

	transit_callback_index = routing.RegisterTransitCallback(time_callback)

	# Define cost of each arc.
	routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

	# Adding capacity constraints.
	# Return amount of carboy at given index.
	def demand_callback(from_index):
		from_node = manager.IndexToNode(from_index)
		return data['demands'][from_node]

	demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
	routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, data['vehicle_capacities'], False, 'Capacity')

	# Setting first solution heuristic.
	search_parameters = pywrapcp.DefaultRoutingSearchParameters()
	search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

	# Solve the problem.
	solution = routing.SolveWithParameters(search_parameters)

	# If solution exists, prepare response json format.
	if solution:
		response = prepare_solution(data, manager, routing, solution)
		return response, 200
	# If doesnt return false.
	else:
		return {'result': False}, 404

if __name__ == '__main__':
	app.run(debug=True)
