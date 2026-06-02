from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class AStarPolicy(RouteController):

    def __init__(self, connection_info):
        super().__init__(connection_info)
        # Pre-compute edge positions for heuristic calculation
        self._compute_edge_positions()

    def _compute_edge_positions(self):
        """Pre-compute the center position of each edge for heuristic calculations"""
        import sumolib
        net = sumolib.net.readNet(self.connection_info.net_filename)
        self.edge_positions = {}
        
        for edge_id in self.connection_info.edge_list:
            edge = net.getEdge(edge_id)
            # Get the shape of the edge (list of coordinates)
            shape = edge.getShape()
            # Use the center point of the edge
            if len(shape) > 0:
                center_x = sum(point[0] for point in shape) / len(shape)
                center_y = sum(point[1] for point in shape) / len(shape)
                self.edge_positions[edge_id] = (center_x, center_y)
            else:
                self.edge_positions[edge_id] = (0, 0)

    def _heuristic(self, edge_a, edge_b):
        """Calculate Euclidean distance heuristic between two edges"""
        if edge_a not in self.edge_positions or edge_b not in self.edge_positions:
            return 0
        
        pos_a = self.edge_positions[edge_a]
        pos_b = self.edge_positions[edge_b]
        return math.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses A* Algorithm to find the shortest path to each individual vehicle's destination
        A* = path cost (g) + heuristic estimate to goal (h)
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            # g_score: actual cost from start to node
            g_score = {edge: 1000000000 for edge in self.connection_info.edge_list}
            # f_score: g_score + heuristic (estimated total cost)
            f_score = {edge: 1000000000 for edge in self.connection_info.edge_list}
            visited = {} # map of visited edges
            current_edge = vehicle.current_edge

            g_score[current_edge] = self.connection_info.edge_length_dict[current_edge]
            f_score[current_edge] = g_score[current_edge] + self._heuristic(current_edge, vehicle.destination)
            
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
            
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                
                current_g = g_score[current_edge]
                
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge in visited:
                        continue
                    
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    tentative_g_score = current_g + edge_length
                    
                    if tentative_g_score < g_score[outgoing_edge]:
                        g_score[outgoing_edge] = tentative_g_score
                        # A* addition: f = g + h
                        f_score[outgoing_edge] = tentative_g_score + self._heuristic(outgoing_edge, vehicle.destination)
                        
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)

                visited[current_edge] = g_score[current_edge]
                
                if current_edge == vehicle.destination:
                    break
                
                # A* change: select node with lowest f_score instead of g_score
                unvisited_edges = [(edge, f_score[edge]) for edge in self.connection_info.edge_list 
                                   if edge not in visited and f_score[edge] < 1000000000]
                
                if not unvisited_edges:
                    break
                
                current_edge, _ = min(unvisited_edges, key=lambda x: x[1])

            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets

