'''
This test file needs the following files:
STR_SUMO.py, RouteController.py, Util.py, test.net.xml, test.rou.xml, myconfig.sumocfg and corresponding SUMO libraries.
'''
from core.STR_SUMO import StrSumo
import os
import sys
import time
from xml.dom.minidom import parse
from core.Util import *
from controller.AStarController import AStarPolicy
from controller.FixedPathPolicy import FixedPathPolicy
from controller.DynamicReroutePolicy import DynamicReroutePolicy
from core.target_vehicles_generation_protocols import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
import traci


HERO_ID = "hero_1"
HERO_START_EDGE = "gneE10"
HERO_DESTINATION_EDGE = "gneE20"
HERO_DEPART_TIME = 20.0  # Give congestion enough time to build before hero enters
HERO_DEADLINE = 1800  # 30 minutes, should be sufficient for any path in this small map

# Direct path (will encounter traffic jam)
HERO_DIRECT_PATH = ["gneE10", "gneE23", "gneE14", "gneE25", "gneE20"]

# Alternative path (detour around congestion)
HERO_ALTERNATIVE_PATH = ["gneE10", "gneE11", "gneE12", "gneE28", "gneE18", "gneE19", "gneE20"]

def inject_hero_vehicle_to_route_file(route_filename, hero_id, start_edge, depart_time, initial_path=None):
    """Adds/updates hero vehicle as an ambulance with a deterministic initial route."""
    dom = parse(route_filename)
    routes_root = dom.documentElement

    # Ensure ambulance vType exists so SUMO GUI renders hero as emergency vehicle.
    hero_type_id = "hero_ambulance"
    existing_type = None
    for vtype_node in routes_root.getElementsByTagName("vType"):
        if vtype_node.getAttribute("id") == hero_type_id:
            existing_type = vtype_node
            break

    if existing_type is None:
        existing_type = dom.createElement("vType")
        existing_type.setAttribute("id", hero_type_id)
        routes_root.insertBefore(existing_type, routes_root.firstChild)

    existing_type.setAttribute("vClass", "emergency")
    existing_type.setAttribute("guiShape", "emergency")
    existing_type.setAttribute("color", "1,1,1")
    existing_type.setAttribute("maxSpeed", "16.67")
    existing_type.setAttribute("speedFactor", "1.0")
    existing_type.setAttribute("speedDev", "0.0")

    # Use full initial path or just start edge.
    route_edges = " ".join(initial_path) if initial_path else start_edge

    # Find existing hero or create it.
    hero_vehicle = None
    for vehicle_node in routes_root.getElementsByTagName("vehicle"):
        if vehicle_node.getAttribute("id") == str(hero_id):
            hero_vehicle = vehicle_node
            break

    if hero_vehicle is None:
        hero_vehicle = dom.createElement("vehicle")
        hero_vehicle.setAttribute("id", str(hero_id))

    hero_vehicle.setAttribute("depart", str(depart_time))
    hero_vehicle.setAttribute("type", hero_type_id)

    # Ensure exactly one route child with deterministic edges.
    route_children = [child for child in hero_vehicle.childNodes if getattr(child, "tagName", None) == "route"]
    if route_children:
        hero_route = route_children[0]
        hero_route.setAttribute("edges", route_edges)
        for extra_route in route_children[1:]:
            hero_vehicle.removeChild(extra_route)
    else:
        hero_route = dom.createElement("route")
        hero_route.setAttribute("edges", route_edges)
        hero_vehicle.appendChild(hero_route)

    if hero_vehicle.parentNode is None:
        # Insert hero in sorted position by departure time.
        vehicles = routes_root.getElementsByTagName("vehicle")
        inserted = False
        for vehicle_node in vehicles:
            node_depart = float(vehicle_node.getAttribute("depart"))
            if node_depart > depart_time:
                routes_root.insertBefore(hero_vehicle, vehicle_node)
                inserted = True
                break

        if not inserted:
            routes_root.appendChild(hero_vehicle)

    with open(route_filename, "w") as route_file:
        route_file.write(dom.toprettyxml())


def inject_congestion_vehicles_to_route_file(route_filename, circulation_path, num_vehicles, start_time, spawn_interval, id_prefix="congestion"):
    """Adds vehicles on the specified path to create traffic jam."""
    dom = parse(route_filename)
    routes_root = dom.documentElement

    # Create path string for congestion vehicles
    route_string = " ".join(circulation_path)
    
    # Add multiple vehicles on the path to create congestion
    for i in range(num_vehicles):
        congestion_vehicle = dom.createElement("vehicle")
        congestion_vehicle.setAttribute("id", f"{id_prefix}_{i}")
        congestion_vehicle.setAttribute("depart", str(start_time + i * spawn_interval))
        
        congestion_route = dom.createElement("route")
        congestion_route.setAttribute("edges", route_string)
        congestion_vehicle.appendChild(congestion_route)
        
        # Keep vehicles in sorted order by departure time
        vehicles = routes_root.getElementsByTagName("vehicle")
        inserted = False
        for vehicle_node in vehicles:
            node_depart = float(vehicle_node.getAttribute("depart"))
            if node_depart > start_time + i * spawn_interval:
                routes_root.insertBefore(congestion_vehicle, vehicle_node)
                inserted = True
                break
        
        if not inserted:
            routes_root.appendChild(congestion_vehicle)

    with open(route_filename, "w") as route_file:
        route_file.write(dom.toprettyxml())


# use vehicle generation protocols to generate vehicle list
def get_controlled_vehicles(route_filename, connection_info, \
    num_controlled_vehicles=10, num_uncontrolled_vehicles=20, pattern = 1):
    '''
    :param @route_filename <str>: the name of the route file to generate
    :param @connection_info <object>: an object that includes the map inforamtion
    :param @num_controlled_vehicles <int>: the number of vehicles controlled by the route controller
    :param @num_uncontrolled_vehicles <int>: the number of vehicles not controlled by the route controller
    :param @pattern <int>: one of four possible patterns. FORMAT:
            -- CASES BEGIN --
                #1. one start point, one destination for all target vehicles
                #2. ranged start point, one destination for all target vehicles
                #3. ranged start points, ranged destination for all target vehicles
            -- CASES ENDS --
    '''
    vehicle_dict = {}
    print(connection_info.net_filename)
    generator = target_vehicles_generator(connection_info.net_filename)

    # Retry generation because Windows can temporarily lock route files.
    max_attempts = 5
    vehicle_list = None
    for attempt in range(1, max_attempts + 1):
        vehicle_list = generator.generate_vehicles(
            num_controlled_vehicles,
            num_uncontrolled_vehicles,
            pattern,
            route_filename,
            connection_info.net_filename,
        )
        if vehicle_list is not None:
            break
        print(f"WARNING: Vehicle generation failed (attempt {attempt}/{max_attempts}). Retrying...")
        time.sleep(0.6)

    # If generation still fails, continue with an empty set so hero-only simulation can run.
    if vehicle_list is None:
        print("WARNING: Proceeding without generated non-hero vehicles due to route file lock.")
        return vehicle_dict

    for vehicle in vehicle_list:
        vehicle_dict[str(vehicle.vehicle_id)] = vehicle

    return vehicle_dict

def test_astar_policy(vehicles):
    print("Testing A* Route Controller")
    scheduler = AStarPolicy(init_connection_info)
    run_simulation(scheduler, vehicles)


def test_fixed_path_policy(vehicles):
    print("Testing FixedPathPolicy for hero vehicle")
    scheduler = FixedPathPolicy(init_connection_info, HERO_ID, HERO_ALTERNATIVE_PATH)
    run_simulation(scheduler, vehicles)


def test_dynamic_reroute_policy(vehicles):
    print("Testing DynamicReroutePolicy - hero will reroute when detecting traffic")
    # Very low threshold: even 1 vehicle on an edge is considered congestion
    scheduler = DynamicReroutePolicy(init_connection_info, HERO_ID, HERO_DIRECT_PATH, HERO_ALTERNATIVE_PATH, congestion_threshold=0.5)
    run_simulation(scheduler, vehicles)


def run_simulation(scheduler, vehicles):

    simulation = StrSumo(scheduler, init_connection_info, vehicles)

    traci.start([sumo_binary, "-c", "./configurations/myconfig.sumocfg", \
                 "--tripinfo-output", "./configurations/trips.trips.xml", \
                 "--fcd-output", "./configurations/testTrace.xml"])

    total_time, end_number, deadlines_missed = simulation.run()
    print("Average timespan: {}, total vehicle number: {}".format(str(total_time/end_number),\
        str(end_number)))
    print(str(deadlines_missed) + ' deadlines missed.')

if __name__ == "__main__":
    # sumo_binary = checkBinary('sumo')  # Non-GUI for testing/debugging
    sumo_binary = checkBinary('sumo-gui')  # GUI for visualization

    # parse config file for map file name
    dom = parse("./configurations/myconfig.sumocfg")

    net_file_node = dom.getElementsByTagName('net-file')
    net_file_attr = net_file_node[0].attributes

    net_file = net_file_attr['value'].nodeValue
    init_connection_info = ConnectionInfo("./configurations/"+net_file)

    route_file_node = dom.getElementsByTagName('route-files')
    route_file_attr = route_file_node[0].attributes
    route_file = "./configurations/"+route_file_attr['value'].nodeValue
    # Minimal random traffic (just 2 vehicles) to avoid interfering with demo
    # All other traffic will be our controlled congestion vehicles on the main path
    vehicles = get_controlled_vehicles(route_file, init_connection_info, 2, 5)

    # Build sustained heavy congestion on the direct route so rerouting has a clear benefit.
    inject_congestion_vehicles_to_route_file(
        route_file,
        ["gneE23", "gneE14", "gneE25", "gneE20"],
        num_vehicles=500,
        start_time=0.0,
        spawn_interval=0.5,
        id_prefix="congestion_direct",
    )

    # Keep the alternate route effectively clear for a fair "escape path" comparison.
    inject_congestion_vehicles_to_route_file(
        route_file,
        HERO_ALTERNATIVE_PATH,
        num_vehicles=1,
        start_time=0.0,
        spawn_interval=10.0,
        id_prefix="congestion_alt",
    )

    # Add one deterministic hero vehicle with initial direct path for reproducible demonstrations.
    # Dynamic routing will override this if congestion is detected
    inject_hero_vehicle_to_route_file(route_file, HERO_ID, HERO_START_EDGE, HERO_DEPART_TIME, HERO_DIRECT_PATH)
    vehicles[HERO_ID] = Vehicle(HERO_ID, HERO_DESTINATION_EDGE, HERO_DEPART_TIME, HERO_DEADLINE)

    #print the controlled vehicles generated
    for vid, v in vehicles.items():
        print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid, \
            v.destination, v.start_time, v.deadline))
    test_dynamic_reroute_policy(vehicles)
