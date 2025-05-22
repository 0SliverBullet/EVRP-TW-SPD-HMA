'''
// -------------------------------------------------------------- -*- python -*-
// File: evrp_tw_spd_validator.py
// --------------------------------------------------------------------------
// Licensed Materials
// Author: Zubin Zheng, Google Gemini 2.5 Pro
// Creation Date: 2025/05/22
// --------------------------------------------------------------------------
//
// evrp_tw_spd_validator.py 
        -- verify the feasibility of a solution for the Electric Vehicle Routing Problem with Time Windows and Single Pickup and Delivery (EVRP-TW-SPD).
        -- epsilon = 1e-2, for float comparisons within the code, i.e., tolerance for floating point errors or constraints validation
        -- The code is designed to parse the problem instance and solution, validate the solution against the problem constraints, and report any discrepancies or errors.
// Example Instance: r202C15
// --------------------------------------------------------------------------
'''
import re
import math

# --- 1. PARSE PROBLEM INSTANCE ---
def parse_evrp_instance(instance_text):
    data = {}
    lines = instance_text.strip().split('\n')
    current_section = None
    
    node_coords_for_dist_calc = {} # Store x,y if needed, though explicit distances are primary

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        if ':' in line and not current_section == "DISTANCETIME_SECTION": # Header lines
            key, value = map(str.strip, line.split(':', 1))
            data[key] = value
            if key == "DIMENSION": data[key] = int(value)
            elif key == "VEHICLES": data[key] = int(value)
            elif key == "DISPATCHINGCOST": data[key] = float(value)
            elif key == "UNITCOST": data[key] = float(value)
            elif key == "CAPACITY": data[key] = float(value)
            elif key == "ELECTRIC_POWER": data[key] = float(value)
            elif key == "CONSUMPTION_RATE": data[key] = float(value)
            elif key == "RECHARGING_RATE": data[key] = float(value)

        elif line == "NODE_SECTION":
            current_section = "NODE_SECTION"
            data['NODES'] = {}
            continue
        elif line == "DISTANCETIME_SECTION":
            current_section = "DISTANCETIME_SECTION"
            data['DISTANCETIME'] = {} # Using a nested dict: dist_time[from][to]
            # First, parse column headers for DISTANCETIME_SECTION
            # Assuming format ID,from_node,to_node,distance,spend_tm
            continue 
        elif line == "DEPOT_SECTION":
            current_section = "DEPOT_SECTION"
            data['DEPOTS'] = []
            continue

        if current_section == "NODE_SECTION":
            if line.startswith("ID,type,x,y"): # Skip header
                continue
            parts = line.split(',')
            node_id = int(parts[0])
            data['NODES'][node_id] = {
                'type': parts[1],
                'x': float(parts[2]),
                'y': float(parts[3]),
                'delivery': float(parts[4]),
                'pickup': float(parts[5]),
                'ready_time': float(parts[6]),
                'due_date': float(parts[7]),
                'service_time': float(parts[8])
            }
            node_coords_for_dist_calc[node_id] = (float(parts[2]), float(parts[3]))

        elif current_section == "DISTANCETIME_SECTION":
            if line.startswith("ID,from_node,to_node"): # Skip header
                continue
            parts = line.split(',')
            # edge_id = int(parts[0]) # Not strictly needed for validation logic
            from_node = int(parts[1])
            to_node = int(parts[2])
            distance = float(parts[3])
            spend_time = float(parts[4])
            
            if from_node not in data['DISTANCETIME']:
                data['DISTANCETIME'][from_node] = {}
            data['DISTANCETIME'][from_node][to_node] = {
                'distance': distance,
                'time': spend_time
            }
            
        elif current_section == "DEPOT_SECTION":
            data['DEPOTS'].append(int(line))
            
    if not data.get('DEPOTS'): # If DEPOT_SECTION is missing but node 0 is type 'd'
        if 0 in data['NODES'] and data['NODES'][0]['type'] == 'd':
            data['DEPOTS'] = [0]
            
    return data

# --- 2. PARSE SOLUTION ---
def parse_solution(solution_text):
    solution = {}
    lines = solution_text.strip().split('\n')
    print(f"Parsing solution with {len(lines)} lines.")
    print(f"Solution lines: {lines}")
    if len(lines) < 2:
        raise ValueError("Invalid solution format: not enough lines.")
    if not lines[0].startswith("Details of the solution"):
        raise ValueError("Invalid solution format: missing 'Solution' header.")
    if not lines[1].startswith("route"):
        raise ValueError("Invalid solution format: missing 'route' line.")
    if not lines[-1].startswith("Total cost:"):
        raise ValueError("Invalid solution format: missing 'Total cost' line.")
    # Extract route details from the second line
    # Example: "route 0, node_num 22, cost 123.45, nodes: 0(0.00, 0.00) 1(0.00, 0.00) ..."
    # Regex to match route details
    print(f"Parsing route line: {lines[1]}")
    route_match = re.match(r"route (\d+), node_num (\d+), cost ([\d.]+), nodes: (.*)", lines[1])
    if not route_match:
        raise ValueError("Invalid route format in solution")
        
    solution['route_id'] = int(route_match.group(1))
    # solution['node_num_reported'] = int(route_match.group(2)) # We'll count actual nodes
    solution['cost_reported'] = float(route_match.group(3))
    nodes_str = route_match.group(4).strip()
    
    parsed_nodes = []
    # Regex to match node ID with optional charge levels: node_id(charge_arrival, charge_departure) or just node_id
    node_pattern = re.compile(r"(\d+)(?:\(([\d.-]+),\s*([\d.-]+)\))?")
    
    last_pos = 0
    for match in node_pattern.finditer(nodes_str):
        node_id = int(match.group(1))
        arrival_charge_sol = float(match.group(2)) if match.group(2) else None
        departure_charge_sol = float(match.group(3)) if match.group(3) else None
        
        parsed_nodes.append({
            'id': node_id,
            'arrival_charge_solution': arrival_charge_sol,
            'departure_charge_solution': departure_charge_sol
        })
        last_pos = match.end()
        # Check for non-whitespace characters between matches, which would indicate an error
        if last_pos < len(nodes_str) and not nodes_str[last_pos:].isspace() and nodes_str[last_pos] not in "0123456789":
             # Handle cases like "10 18(..." vs "1018(..."
             if nodes_str[last_pos] == ' ': # If it's a space, it's fine, continue finding next node
                 pass
             elif nodes_str[last_pos:last_pos+1].isdigit(): # Next element starts with a digit
                 pass
             else: # If there's something else it's an error or unparsed segment
                 remaining_segment = nodes_str[last_pos:].split(' ', 1)[0]
                 if not remaining_segment.isdigit(): # if it's not a simple node id, then it's an error
                    print(f"Warning: Unparsed segment after node {node_id}: '{nodes_str[last_pos:]}'")


    solution['nodes'] = parsed_nodes
    
    # For total cost
    total_cost_match = re.search(r"Total cost: ([\d.]+)", lines[-1])
    if total_cost_match:
        solution['total_cost_reported'] = float(total_cost_match.group(1))
        
    return solution

# --- 3. VALIDATION LOGIC ---
def validate_solution(problem_data, solution_data):
    print("--- Starting Validation ---")
    feasible = True
    errors = []
    epsilon = 1e-2 # For float comparisons

    # Problem parameters
    vehicle_capacity = problem_data['CAPACITY']
    max_battery = problem_data['ELECTRIC_POWER']
    consumption_rate = problem_data['CONSUMPTION_RATE']
    recharging_rate = problem_data['RECHARGING_RATE']
    unit_cost_dist = problem_data['UNITCOST']
    dispatch_cost = problem_data['DISPATCHINGCOST']
    nodes_info = problem_data['NODES']
    dist_time_info = problem_data['DISTANCETIME']
    depot_id = problem_data['DEPOTS'][0]

    # Route details from solution
    route_nodes_sol = solution_data['nodes']
    
    if not route_nodes_sol or route_nodes_sol[0]['id'] != depot_id:
        errors.append(f"Route does not start at depot {depot_id}. Starts at {route_nodes_sol[0]['id'] if route_nodes_sol else 'None'}.")
        feasible = False
    if not route_nodes_sol or route_nodes_sol[-1]['id'] != depot_id:
        errors.append(f"Route does not end at depot {depot_id}. Ends at {route_nodes_sol[-1]['id'] if route_nodes_sol else 'None'}.")
        feasible = False
    
    if not feasible: # Stop if basic route structure is wrong
        for error in errors: print(f"ERROR: {error}")
        print("--- Validation Finished: INFEASIBLE (Basic Structure) ---")
        return False, errors

    # --- Calculate initial load based on all deliveries for this route ---
    # This assumes all 'delivery' amounts for customers on this route are sourced from the depot
    # and loaded onto the vehicle before it departs the depot for the first customer.
    initial_load_from_depot = 0
    if route_nodes_sol: # Check if there are nodes in the solution
        for node_visit_spec in route_nodes_sol:
            node_id = node_visit_spec['id']
            # Ensure node_id is valid and exists in problem_data
            if node_id in nodes_info:
                node_actual_info = nodes_info[node_id]
                if node_actual_info['type'] == 'c': # Only sum deliveries for customer nodes
                    initial_load_from_depot += node_actual_info['delivery']
            else:
                errors.append(f"Node ID {node_id} from solution not found in problem data's NODE_SECTION.")
                feasible = False
                # Stop further processing if a node in solution is invalid
                # (This check might be better placed during solution parsing or very early in validation)

    if not feasible:
        # Handle early exit if node ID issues were found during initial load calculation
        for error in errors: print(f"ERROR: {error}")
        print("--- Validation Finished: INFEASIBLE (Invalid node in solution route) ---")
        return False, errors


    current_time = 0.0
    # current_load = 0.0 # <<-- 原来的初始化
    current_load = initial_load_from_depot # <<-- 新的初始化
    current_charge = 0.0 # Will be set at the first depot visit
    calculated_route_travel_cost = 0.0

    # Initial depot visit (Start of route)
    start_depot_sol = route_nodes_sol[0]
    depot_node_info = nodes_info[depot_id]
    
    current_time = depot_node_info['ready_time'] # Typically 0 for depot
    
    # Set initial charge based on solution: 0(arrival_charge, departure_charge)
    # The arrival_charge for the very first node is effectively the battery state *before* any action.
    # The departure_charge is the state *after* any initial setup/charging at the depot.
    if start_depot_sol['arrival_charge_solution'] is not None:
        # For the very first node, arrival_charge_solution can be seen as the initial state.
        # It's usually full, but we take what the solution provides.
        current_charge = start_depot_sol['arrival_charge_solution']
        if abs(current_charge - max_battery) > epsilon and start_depot_sol['id'] == depot_id:
             print(f"Note: Initial charge at depot {start_depot_sol['id']} is {current_charge}, max is {max_battery}. Solution specifies this state.")
    else: # If not specified, assume full charge at start from depot
        current_charge = max_battery
        print(f"Note: No arrival charge specified for starting depot {depot_id}. Assuming full charge: {current_charge}.")

    if start_depot_sol['departure_charge_solution'] is not None:
        # If departure charge is different, it implies some (usually 0) time spent "charging"
        # or setting up. For the first depot, we often assume departure is immediate with arrival charge.
        if abs(start_depot_sol['departure_charge_solution'] - current_charge) > epsilon:
            # This means solution claims an initial (un)charge at depot before leaving.
            # Let's assume this is an explicit instruction.
            charge_gained = start_depot_sol['departure_charge_solution'] - current_charge
            if charge_gained > 0 and recharging_rate > 0:
                time_to_charge = charge_gained * recharging_rate
                current_time += time_to_charge
                print(f"Note: Depot {depot_id} initial charging from {current_charge:.2f} to {start_depot_sol['departure_charge_solution']:.2f} taking {time_to_charge:.2f} time.")
            elif charge_gained < 0:
                 errors.append(f"Depot {depot_id}: Initial departure charge {start_depot_sol['departure_charge_solution']:.2f} is less than initial arrival charge {current_charge:.2f} without mechanism.")
                 feasible = False
            current_charge = start_depot_sol['departure_charge_solution']
    
    print(f"Start: Depot {depot_id}. Time: {current_time:.2f}, Load: {current_load:.2f}, Charge: {current_charge:.2f}")

    # Iterate through segments of the route
    for i in range(len(route_nodes_sol) - 1):
        from_node_sol = route_nodes_sol[i]
        to_node_sol = route_nodes_sol[i+1]
        
        from_node_id = from_node_sol['id']
        to_node_id = to_node_sol['id']
        
        # Get travel distance and time
        if from_node_id not in dist_time_info or to_node_id not in dist_time_info[from_node_id]:
            errors.append(f"Missing distance/time data for segment {from_node_id} -> {to_node_id}")
            feasible = False
            break 
        segment_dist = dist_time_info[from_node_id][to_node_id]['distance']
        segment_time = dist_time_info[from_node_id][to_node_id]['time']
        
        calculated_route_travel_cost += segment_dist * unit_cost_dist
        
        # --- Battery consumption during travel ---
        energy_consumed = segment_dist * consumption_rate
        current_charge -= energy_consumed
        
        if current_charge < -epsilon: # Allow for small negative due to precision
            errors.append(f"Battery depleted before reaching node {to_node_id} from {from_node_id}. Arrived with {current_charge:.2f} (needed {energy_consumed:.2f}).")
            feasible = False
        
        # --- Arrival at 'to_node' ---
        arrival_time_at_to_node = current_time + segment_time
        node_info_to = nodes_info[to_node_id]
        
        print(f"Travel: {from_node_id} -> {to_node_id}. Dist: {segment_dist:.2f}, TravelTime: {segment_time:.2f}. Consumed: {energy_consumed:.2f}. Arrive at {to_node_id} with Charge: {current_charge:.2f} at Time: {arrival_time_at_to_node:.2f}")

        # --- Solution specified arrival charge check (for charging stations or depot) ---
        if to_node_sol['arrival_charge_solution'] is not None:
            if abs(current_charge - to_node_sol['arrival_charge_solution']) > epsilon: # Increased tolerance for this check
                # This is a significant check for charging stations where the solution dictates battery levels
                errors.append(f"Node {to_node_id} (type {node_info_to['type']}): Calculated arrival charge {current_charge:.2f} differs from solution's arrival charge {to_node_sol['arrival_charge_solution']:.2f}.")
                feasible = False
                # Option: override with solution's value if you trust it for further checks, but mark as error
                # current_charge = to_node_sol['arrival_charge_solution'] 
        
        # --- Time Window ---
        service_start_time = max(arrival_time_at_to_node, node_info_to['ready_time'])
        
        if service_start_time > node_info_to['due_date'] + epsilon:
            errors.append(f"Node {to_node_id}: Arrival for service ({service_start_time:.2f}) is after due date ({node_info_to['due_date']:.2f}). Arrived at node: {arrival_time_at_to_node:.2f}, Ready: {node_info_to['ready_time']:.2f}.")
            feasible = False

        # --- Node specific actions (service, charging) ---
        actual_service_or_charge_time = 0
        
        if node_info_to['type'] == 'c': # Customer
            actual_service_or_charge_time = node_info_to['service_time']
            service_end_time = service_start_time + actual_service_or_charge_time
            
            # if service_end_time > node_info_to['due_date'] + epsilon:
            #     errors.append(f"Node {to_node_id}: Service end ({service_end_time:.2f}) is after due date ({node_info_to['due_date']:.2f}). Service started at {service_start_time:.2f}.")
            #     feasible = False

            # # Capacity update
            # current_load -= node_info_to['delivery']
            # current_load += node_info_to['pickup']

            # if current_load > vehicle_capacity + epsilon or current_load < -epsilon:
            #     errors.append(f"Node {to_node_id}: Capacity violation. Load: {current_load:.2f}, Capacity: {vehicle_capacity:.2f}.")
            #     feasible = False
            
            # <<--- 核心修改在这里如何影响负载更新和检查 --- >>
            # 车辆到达客户点，先进行配送（卸货）
            delivery_at_node = node_info_to['delivery']
            if current_load < delivery_at_node - epsilon: # 检查是否有足够的货物进行配送
                errors.append(f"Node {to_node_id}: Insufficient load for delivery. Current Load: {current_load:.2f}, Delivery Demand: {delivery_at_node:.2f}.")
                feasible = False
            current_load -= delivery_at_node # 卸货

            # 然后进行取货（装货）
            pickup_at_node = node_info_to['pickup']
            current_load += pickup_at_node # 装货
            
            # 检查车辆容量是否超出
            if current_load > vehicle_capacity + epsilon:
                errors.append(f"Node {to_node_id}: Capacity violation after pickup. Load: {current_load:.2f}, Capacity: {vehicle_capacity:.2f}.")
                feasible = False
            
            # （可选）检查 current_load 是否为负。如果初始装载正确，且delivery/pickup非负，
            # 并且上面对 delivery_at_node 的检查通过了，这里 current_load 一般不会为负。
            # 但为保险起见，可以保留或调整 `current_load < -epsilon` 的检查。
            # 如果 current_load 严格代表物理量，它不应小于0。
            if current_load < -epsilon: # 确保负载在逻辑上仍然合理 (不应发生如果上面检查完善)
                 errors.append(f"Node {to_node_id}: Negative load occurred ({current_load:.2f}). This indicates a logical flaw or data issue.")
                 feasible = False
            
            current_time = service_end_time
            # Charge does not change during customer service unless specified otherwise
            print(f"  Service Customer {to_node_id}: Start: {service_start_time:.2f}, End: {service_end_time:.2f}. Load: {current_load:.2f}. DepartCharge: {current_charge:.2f}")

        elif node_info_to['type'] == 'f': # Charging Station
            if to_node_sol['departure_charge_solution'] is None:
                errors.append(f"Charging station {to_node_id}: Solution does not specify departure charge.")
                feasible = False
                # Cannot proceed without knowing target charge
            else:
                departure_charge_target_sol = to_node_sol['departure_charge_solution']
                if departure_charge_target_sol > max_battery + epsilon:
                    errors.append(f"Charging station {to_node_id}: Attempting to charge ({departure_charge_target_sol:.2f}) beyond max capacity ({max_battery:.2f}).")
                    feasible = False
                
                charge_to_gain = departure_charge_target_sol - current_charge # current_charge is arrival charge
                
                if charge_to_gain < -epsilon : # Cannot "gain" negative charge
                    errors.append(f"Charging station {to_node_id}: Departure charge ({departure_charge_target_sol:.2f}) is less than arrival charge ({current_charge:.2f}).")
                    feasible = False
                
                if recharging_rate <= 0 and charge_to_gain > epsilon:
                     errors.append(f"Charging station {to_node_id}: Recharging rate is {recharging_rate}, cannot gain charge {charge_to_gain:.2f}.")
                     feasible = False
                
                if recharging_rate > 0 and charge_to_gain > 0: #Only positive gain requires time
                    actual_service_or_charge_time = charge_to_gain * recharging_rate
                else: # if no charge needed, or target is same as arrival
                    actual_service_or_charge_time = 0 # Or problem's base service_time for station if any

                # Add fixed service time for station if any (usually 0 for EVRP, time is for charging)
                actual_service_or_charge_time += node_info_to['service_time'] 
                
                service_end_time = service_start_time + actual_service_or_charge_time
                
                # if service_end_time > node_info_to['due_date'] + epsilon:
                #     errors.append(f"Node {to_node_id} (Station): Service/Charging end ({service_end_time:.2f}) is after due date ({node_info_to['due_date']:.2f}).")
                #     feasible = False
                
                current_time = service_end_time
                current_charge = departure_charge_target_sol # Set to solution's target
                print(f"  Charge Station {to_node_id}: Start: {service_start_time:.2f}, End: {service_end_time:.2f} (Charged for {actual_service_or_charge_time:.2f}). DepartCharge: {current_charge:.2f}")

        elif node_info_to['type'] == 'd': # Depot (this will be the final depot visit)
            # Check arrival time at depot
            if arrival_time_at_to_node > node_info_to['due_date'] + epsilon:
                errors.append(f"Final Depot {to_node_id}: Arrival ({arrival_time_at_to_node:.2f}) is after due date ({node_info_to['due_date']:.2f}).")
                feasible = False
            
            # Final charge check. Solution says "0(-0.00, -0.00)". Arrival is -0.00.
            if to_node_sol['arrival_charge_solution'] is not None:
                 if abs(current_charge - to_node_sol['arrival_charge_solution']) > epsilon: # Allow some tolerance
                    errors.append(f"Final Depot {to_node_id}: Calculated arrival charge {current_charge:.2f} differs from solution's arrival {to_node_sol['arrival_charge_solution']:.2f}.")
                    # feasible = False # This might be a soft check depending on problem spec for final depot
            
            # Load should be 0 when returning to depot if all pickups/deliveries are balanced for the route
            # (This problem type SPD may not require load to be 0 if it's a single tour not part of a larger VRP)
            # For a single route, this often means net deliveries are done.
            # If we interpret capacity strictly that it returns empty, then:
            # if abs(current_load) > epsilon:
            #    errors.append(f"Final Depot {to_node_id}: Final load is {current_load:.2f}, expected 0.")
            #    feasible = False
            
            current_time = arrival_time_at_to_node # No service at final depot in this loop
            print(f"  Return to Depot {to_node_id}: ArrivalTime: {current_time:.2f}. FinalLoad: {current_load:.2f}. FinalCharge: {current_charge:.2f}")
        
        if not feasible: # If any check within the loop fails
            break 
            
    # --- Cost Validation ---
    # The solution cost is just travel cost for one vehicle.
    # Total cost = route_travel_cost + dispatch_cost
    
    calculated_total_cost = calculated_route_travel_cost + dispatch_cost # Assuming 1 vehicle
    
    if abs(calculated_route_travel_cost - solution_data['cost_reported']) > epsilon: # Allow 0.1 absolute or relative
        errors.append(f"Route cost mismatch. Calculated: {calculated_route_travel_cost:.2f}, Reported: {solution_data['cost_reported']:.2f}.")
        feasible = False # This is often a strong indicator of other issues or different cost model

    if 'total_cost_reported' in solution_data:
        if abs(calculated_total_cost - solution_data['total_cost_reported']) > epsilon:
             errors.append(f"Total cost mismatch. Calculated: {calculated_total_cost:.2f}, Reported: {solution_data['total_cost_reported']:.2f}.")
             feasible = False
    else:
        print("Warning: Total cost not explicitly reported in solution details to compare against.")


    if not errors:
        print("Solution appears FEASIBLE.")
    else:
        print("\n--- Validation Finished: INFEASIBLE or Discrepancies Found ---")
        for error in errors:
            print(f"ERROR: {error}")
            
    return feasible and not errors, errors


# --- Main Execution ---
if __name__ == "__main__":
    instance_text = """
NAME : r202C15 
 TYPE : EVRP-TW-SPD 
 DIMENSION : 22 
 VEHICLES : 25 
 DISPATCHINGCOST : 1000 
 UNITCOST : 1.0 
 CAPACITY : 1000.0 
 ELECTRIC_POWER : 60.63 
 CONSUMPTION_RATE : 1.0 
 RECHARGING_RATE : 0.49 
 EDGE_WEIGHT_TYPE : EXPLICIT 
 NODE_SECTION 
 ID,type,x,y,delivery,pickup,ready_time,due_date,service_time 
 0,d,35.0,35.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 1,c,57.0,48.0,19.000000000000000,4.000000000000000,0.0,964.0,10.0 
 2,c,11.0,14.0,14.000000000000000,4.000000000000000,42.0,178.0,10.0 
 3,c,55.0,5.0,2.000000000000000,27.000000000000000,53.0,233.0,10.0 
 4,c,5.0,5.0,16.000000000000000,0.000000000000000,0.0,947.0,10.0 
 5,c,65.0,20.0,1.000000000000000,5.000000000000000,418.0,532.0,10.0 
 6,c,42.0,7.0,0.000000000000000,5.000000000000000,782.0,912.0,10.0 
 7,c,37.0,56.0,3.000000000000000,2.000000000000000,0.0,968.0,10.0 
 8,c,16.0,22.0,29.000000000000000,12.000000000000000,552.0,744.0,10.0 
 9,c,28.0,18.0,16.000000000000000,10.000000000000000,530.0,568.0,10.0 
 10,c,2.0,48.0,0.000000000000000,1.000000000000000,46.0,182.0,10.0 
 11,c,12.0,24.0,6.000000000000000,7.000000000000000,329.0,551.0,10.0 
 12,c,13.0,52.0,9.000000000000000,27.000000000000000,418.0,558.0,10.0 
 13,c,24.0,12.0,2.000000000000000,3.000000000000000,0.0,964.0,10.0 
 14,c,47.0,16.0,8.000000000000000,17.000000000000000,104.0,252.0,10.0 
 15,c,20.0,65.0,3.000000000000000,9.000000000000000,0.0,956.0,10.0 
 16,f,35.0,35.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 17,f,28.0,62.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 18,f,15.0,42.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 19,f,21.0,22.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 20,f,34.0,16.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 21,f,64.0,19.0,0.000000000000000,0.000000000000000,0.0,1000.0,0.0 
 DISTANCETIME_SECTION 
 ID,from_node,to_node,distance,spend_tm 
 0,0,1,25.553864678361276,25.553864678361276 
 1,0,2,31.890437438203950,31.890437438203950 
 2,0,3,36.055512754639892,36.055512754639892 
 3,0,4,42.426406871192853,42.426406871192853 
 4,0,5,33.541019662496844,33.541019662496844 
 5,0,6,28.861739379323623,28.861739379323623 
 6,0,7,21.095023109728988,21.095023109728988 
 7,0,8,23.021728866442675,23.021728866442675 
 8,0,9,18.384776310850235,18.384776310850235 
 9,0,10,35.468295701936398,35.468295701936398 
 10,0,11,25.495097567963924,25.495097567963924 
 11,0,12,27.802877548915689,27.802877548915689 
 12,0,13,25.495097567963924,25.495097567963924 
 13,0,14,22.472205054244231,22.472205054244231 
 14,0,15,33.541019662496844,33.541019662496844 
 15,0,16,0.000000000000000,0.000000000000000 
 16,0,17,27.892651361962706,27.892651361962706 
 17,0,18,21.189620100417091,21.189620100417091 
 18,0,19,19.104973174542799,19.104973174542799 
 19,0,20,19.026297590440446,19.026297590440446 
 20,0,21,33.120990323358392,33.120990323358392 
 21,1,0,25.553864678361276,25.553864678361276 
 22,1,2,57.201398584300364,57.201398584300364 
 23,1,3,43.046486500061768,43.046486500061768 
 24,1,4,67.475921631349351,67.475921631349351 
 25,1,5,29.120439557122072,29.120439557122072 
 26,1,6,43.657759905886145,43.657759905886145 
 27,1,7,21.540659228538015,21.540659228538015 
 28,1,8,48.548944375753422,48.548944375753422 
 29,1,9,41.725292090050132,41.725292090050132 
 30,1,10,55.000000000000000,55.000000000000000 
 31,1,11,51.000000000000000,51.000000000000000 
 32,1,12,44.181444068749045,44.181444068749045 
 33,1,13,48.836461788299118,48.836461788299118 
 34,1,14,33.526109228480422,33.526109228480422 
 35,1,15,40.718546143004666,40.718546143004666 
 36,1,16,25.553864678361276,25.553864678361276 
 37,1,17,32.202484376209235,32.202484376209235 
 38,1,18,42.426406871192853,42.426406871192853 
 39,1,19,44.407206622349037,44.407206622349037 
 40,1,20,39.408120990476064,39.408120990476064 
 41,1,21,29.832867780352597,29.832867780352597 
 42,2,0,31.890437438203950,31.890437438203950 
 43,2,1,57.201398584300364,57.201398584300364 
 44,2,3,44.911023145771239,44.911023145771239 
 45,2,4,10.816653826391969,10.816653826391969 
 46,2,5,54.332310828824497,54.332310828824497 
 47,2,6,31.780497164141408,31.780497164141408 
 48,2,7,49.396356140913873,49.396356140913873 
 49,2,8,9.433981132056603,9.433981132056603 
 50,2,9,17.464249196572979,17.464249196572979 
 51,2,10,35.171010790137949,35.171010790137949 
 52,2,11,10.049875621120890,10.049875621120890 
 53,2,12,38.052595180880893,38.052595180880893 
 54,2,13,13.152946437965905,13.152946437965905 
 55,2,14,36.055512754639892,36.055512754639892 
 56,2,15,51.788029504896208,51.788029504896208 
 57,2,16,31.890437438203950,31.890437438203950 
 58,2,17,50.921508225896062,50.921508225896062 
 59,2,18,28.284271247461902,28.284271247461902 
 60,2,19,12.806248474865697,12.806248474865697 
 61,2,20,23.086792761230392,23.086792761230392 
 62,2,21,53.235326616824658,53.235326616824658 
 63,3,0,36.055512754639892,36.055512754639892 
 64,3,1,43.046486500061768,43.046486500061768 
 65,3,2,44.911023145771239,44.911023145771239 
 66,3,4,50.000000000000000,50.000000000000000 
 67,3,5,18.027756377319946,18.027756377319946 
 68,3,6,13.152946437965905,13.152946437965905 
 69,3,7,54.083269131959838,54.083269131959838 
 70,3,8,42.544094772365298,42.544094772365298 
 71,3,9,29.966648127543394,29.966648127543394 
 72,3,10,68.249542123006222,68.249542123006222 
 73,3,11,47.010637094172637,47.010637094172637 
 74,3,12,63.031738037277698,63.031738037277698 
 75,3,13,31.780497164141408,31.780497164141408 
 76,3,14,13.601470508735444,13.601470508735444 
 77,3,15,69.462219947249025,69.462219947249025 
 78,3,16,36.055512754639892,36.055512754639892 
 79,3,17,63.071388124885914,63.071388124885914 
 80,3,18,54.488530903301111,54.488530903301111 
 81,3,19,38.013155617496423,38.013155617496423 
 82,3,20,23.706539182259394,23.706539182259394 
 83,3,21,16.643316977093239,16.643316977093239 
 84,4,0,42.426406871192853,42.426406871192853 
 85,4,1,67.475921631349351,67.475921631349351 
 86,4,2,10.816653826391969,10.816653826391969 
 87,4,3,50.000000000000000,50.000000000000000 
 88,4,5,61.846584384264908,61.846584384264908 
 89,4,6,37.054014627297811,37.054014627297811 
 90,4,7,60.207972893961475,60.207972893961475 
 91,4,8,20.248456731316587,20.248456731316587 
 92,4,9,26.419689627245813,26.419689627245813 
 93,4,10,43.104524124504614,43.104524124504614 
 94,4,11,20.248456731316587,20.248456731316587 
 95,4,12,47.675989764240867,47.675989764240867 
 96,4,13,20.248456731316587,20.248456731316587 
 97,4,14,43.416586692184822,43.416586692184822 
 98,4,15,61.846584384264908,61.846584384264908 
 99,4,16,42.426406871192853,42.426406871192853 
 100,4,17,61.465437442517235,61.465437442517235 
 101,4,18,38.327535793473601,38.327535793473601 
 102,4,19,23.345235059857504,23.345235059857504 
 103,4,20,31.016124838541646,31.016124838541646 
 104,4,21,60.638271743182131,60.638271743182131 
 105,5,0,33.541019662496844,33.541019662496844 
 106,5,1,29.120439557122072,29.120439557122072 
 107,5,2,54.332310828824497,54.332310828824497 
 108,5,3,18.027756377319946,18.027756377319946 
 109,5,4,61.846584384264908,61.846584384264908 
 110,5,6,26.419689627245813,26.419689627245813 
 111,5,7,45.607017003965517,45.607017003965517 
 112,5,8,49.040799340956916,49.040799340956916 
 113,5,9,37.054014627297811,37.054014627297811 
 114,5,10,68.942004612572731,68.942004612572731 
 115,5,11,53.150729063673246,53.150729063673246 
 116,5,12,61.057350089894989,61.057350089894989 
 117,5,13,41.773197148410844,41.773197148410844 
 118,5,14,18.439088914585774,18.439088914585774 
 119,5,15,63.639610306789280,63.639610306789280 
 120,5,16,33.541019662496844,33.541019662496844 
 121,5,17,55.973207876626120,55.973207876626120 
 122,5,18,54.626001134990652,54.626001134990652 
 123,5,19,44.045431091090478,44.045431091090478 
 124,5,20,31.256999216175569,31.256999216175569 
 125,5,21,1.414213562373095,1.414213562373095 
 126,6,0,28.861739379323623,28.861739379323623 
 127,6,1,43.657759905886145,43.657759905886145 
 128,6,2,31.780497164141408,31.780497164141408 
 129,6,3,13.152946437965905,13.152946437965905 
 130,6,4,37.054014627297811,37.054014627297811 
 131,6,5,26.419689627245813,26.419689627245813 
 132,6,7,49.254441424099006,49.254441424099006 
 133,6,8,30.016662039607269,30.016662039607269 
 134,6,9,17.804493814764857,17.804493814764857 
 135,6,10,57.280013966478741,57.280013966478741 
 136,6,11,34.481879299133332,34.481879299133332 
 137,6,12,53.535035257296691,53.535035257296691 
 138,6,13,18.681541692269406,18.681541692269406 
 139,6,14,10.295630140987001,10.295630140987001 
 140,6,15,62.032249677083293,62.032249677083293 
 141,6,16,28.861739379323623,28.861739379323623 
 142,6,17,56.753854494650845,56.753854494650845 
 143,6,18,44.204072210600685,44.204072210600685 
 144,6,19,25.806975801127880,25.806975801127880 
 145,6,20,12.041594578792296,12.041594578792296 
 146,6,21,25.059928172283335,25.059928172283335 
 147,7,0,21.095023109728988,21.095023109728988 
 148,7,1,21.540659228538015,21.540659228538015 
 149,7,2,49.396356140913873,49.396356140913873 
 150,7,3,54.083269131959838,54.083269131959838 
 151,7,4,60.207972893961475,60.207972893961475 
 152,7,5,45.607017003965517,45.607017003965517 
 153,7,6,49.254441424099006,49.254441424099006 
 154,7,8,39.962482405376171,39.962482405376171 
 155,7,9,39.051248379533270,39.051248379533270 
 156,7,10,35.902646142032481,35.902646142032481 
 157,7,11,40.607881008493905,40.607881008493905 
 158,7,12,24.331050121192877,24.331050121192877 
 159,7,13,45.880278987817846,45.880278987817846 
 160,7,14,41.231056256176608,41.231056256176608 
 161,7,15,19.235384061671343,19.235384061671343 
 162,7,16,21.095023109728988,21.095023109728988 
 163,7,17,10.816653826391969,10.816653826391969 
 164,7,18,26.076809620810597,26.076809620810597 
 165,7,19,37.576588456111871,37.576588456111871 
 166,7,20,40.112342240263160,40.112342240263160 
 167,7,21,45.803929962395145,45.803929962395145 
 168,8,0,23.021728866442675,23.021728866442675 
 169,8,1,48.548944375753422,48.548944375753422 
 170,8,2,9.433981132056603,9.433981132056603 
 171,8,3,42.544094772365298,42.544094772365298 
 172,8,4,20.248456731316587,20.248456731316587 
 173,8,5,49.040799340956916,49.040799340956916 
 174,8,6,30.016662039607269,30.016662039607269 
 175,8,7,39.962482405376171,39.962482405376171 
 176,8,9,12.649110640673518,12.649110640673518 
 177,8,10,29.529646120466801,29.529646120466801 
 178,8,11,4.472135954999580,4.472135954999580 
 179,8,12,30.149626863362670,30.149626863362670 
 180,8,13,12.806248474865697,12.806248474865697 
 181,8,14,31.575306807693888,31.575306807693888 
 182,8,15,43.185645763378368,43.185645763378368 
 183,8,16,23.021728866442675,23.021728866442675 
 184,8,17,41.761226035642203,41.761226035642203 
 185,8,18,20.024984394500787,20.024984394500787 
 186,8,19,5.000000000000000,5.000000000000000 
 187,8,20,18.973665961010276,18.973665961010276 
 188,8,21,48.093658625644196,48.093658625644196 
 189,9,0,18.384776310850235,18.384776310850235 
 190,9,1,41.725292090050132,41.725292090050132 
 191,9,2,17.464249196572979,17.464249196572979 
 192,9,3,29.966648127543394,29.966648127543394 
 193,9,4,26.419689627245813,26.419689627245813 
 194,9,5,37.054014627297811,37.054014627297811 
 195,9,6,17.804493814764857,17.804493814764857 
 196,9,7,39.051248379533270,39.051248379533270 
 197,9,8,12.649110640673518,12.649110640673518 
 198,9,10,39.698866482558415,39.698866482558415 
 199,9,11,17.088007490635061,17.088007490635061 
 200,9,12,37.161808352124091,37.161808352124091 
 201,9,13,7.211102550927978,7.211102550927978 
 202,9,14,19.104973174542799,19.104973174542799 
 203,9,15,47.675989764240867,47.675989764240867 
 204,9,16,18.384776310850235,18.384776310850235 
 205,9,17,44.000000000000000,44.000000000000000 
 206,9,18,27.294688127912362,27.294688127912362 
 207,9,19,8.062257748298549,8.062257748298549 
 208,9,20,6.324555320336759,6.324555320336759 
 209,9,21,36.013886210738214,36.013886210738214 
 210,10,0,35.468295701936398,35.468295701936398 
 211,10,1,55.000000000000000,55.000000000000000 
 212,10,2,35.171010790137949,35.171010790137949 
 213,10,3,68.249542123006222,68.249542123006222 
 214,10,4,43.104524124504614,43.104524124504614 
 215,10,5,68.942004612572731,68.942004612572731 
 216,10,6,57.280013966478741,57.280013966478741 
 217,10,7,35.902646142032481,35.902646142032481 
 218,10,8,29.529646120466801,29.529646120466801 
 219,10,9,39.698866482558415,39.698866482558415 
 220,10,11,26.000000000000000,26.000000000000000 
 221,10,12,11.704699910719626,11.704699910719626 
 222,10,13,42.190046219457976,42.190046219457976 
 223,10,14,55.217750769114090,55.217750769114090 
 224,10,15,24.758836806279895,24.758836806279895 
 225,10,16,35.468295701936398,35.468295701936398 
 226,10,17,29.529646120466801,29.529646120466801 
 227,10,18,14.317821063276353,14.317821063276353 
 228,10,19,32.202484376209235,32.202484376209235 
 229,10,20,45.254833995939045,45.254833995939045 
 230,10,21,68.447059834590405,68.447059834590405 
 231,11,0,25.495097567963924,25.495097567963924 
 232,11,1,51.000000000000000,51.000000000000000 
 233,11,2,10.049875621120890,10.049875621120890 
 234,11,3,47.010637094172637,47.010637094172637 
 235,11,4,20.248456731316587,20.248456731316587 
 236,11,5,53.150729063673246,53.150729063673246 
 237,11,6,34.481879299133332,34.481879299133332 
 238,11,7,40.607881008493905,40.607881008493905 
 239,11,8,4.472135954999580,4.472135954999580 
 240,11,9,17.088007490635061,17.088007490635061 
 241,11,10,26.000000000000000,26.000000000000000 
 242,11,12,28.017851452243800,28.017851452243800 
 243,11,13,16.970562748477139,16.970562748477139 
 244,11,14,35.902646142032481,35.902646142032481 
 245,11,15,41.773197148410844,41.773197148410844 
 246,11,16,25.495097567963924,25.495097567963924 
 247,11,17,41.231056256176608,41.231056256176608 
 248,11,18,18.248287590894659,18.248287590894659 
 249,11,19,9.219544457292887,9.219544457292887 
 250,11,20,23.409399821439251,23.409399821439251 
 251,11,21,52.239831546435909,52.239831546435909 
 252,12,0,27.802877548915689,27.802877548915689 
 253,12,1,44.181444068749045,44.181444068749045 
 254,12,2,38.052595180880893,38.052595180880893 
 255,12,3,63.031738037277698,63.031738037277698 
 256,12,4,47.675989764240867,47.675989764240867 
 257,12,5,61.057350089894989,61.057350089894989 
 258,12,6,53.535035257296691,53.535035257296691 
 259,12,7,24.331050121192877,24.331050121192877 
 260,12,8,30.149626863362670,30.149626863362670 
 261,12,9,37.161808352124091,37.161808352124091 
 262,12,10,11.704699910719626,11.704699910719626 
 263,12,11,28.017851452243800,28.017851452243800 
 264,12,13,41.484937025383083,41.484937025383083 
 265,12,14,49.517673612559790,49.517673612559790 
 266,12,15,14.764823060233400,14.764823060233400 
 267,12,16,27.802877548915689,27.802877548915689 
 268,12,17,18.027756377319946,18.027756377319946 
 269,12,18,10.198039027185569,10.198039027185569 
 270,12,19,31.048349392520048,31.048349392520048 
 271,12,20,41.677331968349414,41.677331968349414 
 272,12,21,60.745370193949761,60.745370193949761 
 273,13,0,25.495097567963924,25.495097567963924 
 274,13,1,48.836461788299118,48.836461788299118 
 275,13,2,13.152946437965905,13.152946437965905 
 276,13,3,31.780497164141408,31.780497164141408 
 277,13,4,20.248456731316587,20.248456731316587 
 278,13,5,41.773197148410844,41.773197148410844 
 279,13,6,18.681541692269406,18.681541692269406 
 280,13,7,45.880278987817846,45.880278987817846 
 281,13,8,12.806248474865697,12.806248474865697 
 282,13,9,7.211102550927978,7.211102550927978 
 283,13,10,42.190046219457976,42.190046219457976 
 284,13,11,16.970562748477139,16.970562748477139 
 285,13,12,41.484937025383083,41.484937025383083 
 286,13,14,23.345235059857504,23.345235059857504 
 287,13,15,53.150729063673246,53.150729063673246 
 288,13,16,25.495097567963924,25.495097567963924 
 289,13,17,50.159744815937813,50.159744815937813 
 290,13,18,31.320919526731650,31.320919526731650 
 291,13,19,10.440306508910551,10.440306508910551 
 292,13,20,10.770329614269007,10.770329614269007 
 293,13,21,40.607881008493905,40.607881008493905 
 294,14,0,22.472205054244231,22.472205054244231 
 295,14,1,33.526109228480422,33.526109228480422 
 296,14,2,36.055512754639892,36.055512754639892 
 297,14,3,13.601470508735444,13.601470508735444 
 298,14,4,43.416586692184822,43.416586692184822 
 299,14,5,18.439088914585774,18.439088914585774 
 300,14,6,10.295630140987001,10.295630140987001 
 301,14,7,41.231056256176608,41.231056256176608 
 302,14,8,31.575306807693888,31.575306807693888 
 303,14,9,19.104973174542799,19.104973174542799 
 304,14,10,55.217750769114090,55.217750769114090 
 305,14,11,35.902646142032481,35.902646142032481 
 306,14,12,49.517673612559790,49.517673612559790 
 307,14,13,23.345235059857504,23.345235059857504 
 308,14,15,55.946402922797461,55.946402922797461 
 309,14,16,22.472205054244231,22.472205054244231 
 310,14,17,49.769468552517218,49.769468552517218 
 311,14,18,41.231056256176608,41.231056256176608 
 312,14,19,26.683328128252668,26.683328128252668 
 313,14,20,13.000000000000000,13.000000000000000 
 314,14,21,17.262676501632068,17.262676501632068 
 315,15,0,33.541019662496844,33.541019662496844 
 316,15,1,40.718546143004666,40.718546143004666 
 317,15,2,51.788029504896208,51.788029504896208 
 318,15,3,69.462219947249025,69.462219947249025 
 319,15,4,61.846584384264908,61.846584384264908 
 320,15,5,63.639610306789280,63.639610306789280 
 321,15,6,62.032249677083293,62.032249677083293 
 322,15,7,19.235384061671343,19.235384061671343 
 323,15,8,43.185645763378368,43.185645763378368 
 324,15,9,47.675989764240867,47.675989764240867 
 325,15,10,24.758836806279895,24.758836806279895 
 326,15,11,41.773197148410844,41.773197148410844 
 327,15,12,14.764823060233400,14.764823060233400 
 328,15,13,53.150729063673246,53.150729063673246 
 329,15,14,55.946402922797461,55.946402922797461 
 330,15,16,33.541019662496844,33.541019662496844 
 331,15,17,8.544003745317530,8.544003745317530 
 332,15,18,23.537204591879640,23.537204591879640 
 333,15,19,43.011626335213137,43.011626335213137 
 334,15,20,50.960769224963627,50.960769224963627 
 335,15,21,63.655321851358195,63.655321851358195 
 336,16,0,0.000000000000000,0.000000000000000 
 337,16,1,25.553864678361276,25.553864678361276 
 338,16,2,31.890437438203950,31.890437438203950 
 339,16,3,36.055512754639892,36.055512754639892 
 340,16,4,42.426406871192853,42.426406871192853 
 341,16,5,33.541019662496844,33.541019662496844 
 342,16,6,28.861739379323623,28.861739379323623 
 343,16,7,21.095023109728988,21.095023109728988 
 344,16,8,23.021728866442675,23.021728866442675 
 345,16,9,18.384776310850235,18.384776310850235 
 346,16,10,35.468295701936398,35.468295701936398 
 347,16,11,25.495097567963924,25.495097567963924 
 348,16,12,27.802877548915689,27.802877548915689 
 349,16,13,25.495097567963924,25.495097567963924 
 350,16,14,22.472205054244231,22.472205054244231 
 351,16,15,33.541019662496844,33.541019662496844 
 352,16,17,27.892651361962706,27.892651361962706 
 353,16,18,21.189620100417091,21.189620100417091 
 354,16,19,19.104973174542799,19.104973174542799 
 355,16,20,19.026297590440446,19.026297590440446 
 356,16,21,33.120990323358392,33.120990323358392 
 357,17,0,27.892651361962706,27.892651361962706 
 358,17,1,32.202484376209235,32.202484376209235 
 359,17,2,50.921508225896062,50.921508225896062 
 360,17,3,63.071388124885914,63.071388124885914 
 361,17,4,61.465437442517235,61.465437442517235 
 362,17,5,55.973207876626120,55.973207876626120 
 363,17,6,56.753854494650845,56.753854494650845 
 364,17,7,10.816653826391969,10.816653826391969 
 365,17,8,41.761226035642203,41.761226035642203 
 366,17,9,44.000000000000000,44.000000000000000 
 367,17,10,29.529646120466801,29.529646120466801 
 368,17,11,41.231056256176608,41.231056256176608 
 369,17,12,18.027756377319946,18.027756377319946 
 370,17,13,50.159744815937813,50.159744815937813 
 371,17,14,49.769468552517218,49.769468552517218 
 372,17,15,8.544003745317530,8.544003745317530 
 373,17,16,27.892651361962706,27.892651361962706 
 374,17,18,23.853720883753127,23.853720883753127 
 375,17,19,40.607881008493905,40.607881008493905 
 376,17,20,46.389654018972806,46.389654018972806 
 377,17,21,56.080299571239813,56.080299571239813 
 378,18,0,21.189620100417091,21.189620100417091 
 379,18,1,42.426406871192853,42.426406871192853 
 380,18,2,28.284271247461902,28.284271247461902 
 381,18,3,54.488530903301111,54.488530903301111 
 382,18,4,38.327535793473601,38.327535793473601 
 383,18,5,54.626001134990652,54.626001134990652 
 384,18,6,44.204072210600685,44.204072210600685 
 385,18,7,26.076809620810597,26.076809620810597 
 386,18,8,20.024984394500787,20.024984394500787 
 387,18,9,27.294688127912362,27.294688127912362 
 388,18,10,14.317821063276353,14.317821063276353 
 389,18,11,18.248287590894659,18.248287590894659 
 390,18,12,10.198039027185569,10.198039027185569 
 391,18,13,31.320919526731650,31.320919526731650 
 392,18,14,41.231056256176608,41.231056256176608 
 393,18,15,23.537204591879640,23.537204591879640 
 394,18,16,21.189620100417091,21.189620100417091 
 395,18,17,23.853720883753127,23.853720883753127 
 396,18,19,20.880613017821101,20.880613017821101 
 397,18,20,32.202484376209235,32.202484376209235 
 398,18,21,54.129474410897430,54.129474410897430 
 399,19,0,19.104973174542799,19.104973174542799 
 400,19,1,44.407206622349037,44.407206622349037 
 401,19,2,12.806248474865697,12.806248474865697 
 402,19,3,38.013155617496423,38.013155617496423 
 403,19,4,23.345235059857504,23.345235059857504 
 404,19,5,44.045431091090478,44.045431091090478 
 405,19,6,25.806975801127880,25.806975801127880 
 406,19,7,37.576588456111871,37.576588456111871 
 407,19,8,5.000000000000000,5.000000000000000 
 408,19,9,8.062257748298549,8.062257748298549 
 409,19,10,32.202484376209235,32.202484376209235 
 410,19,11,9.219544457292887,9.219544457292887 
 411,19,12,31.048349392520048,31.048349392520048 
 412,19,13,10.440306508910551,10.440306508910551 
 413,19,14,26.683328128252668,26.683328128252668 
 414,19,15,43.011626335213137,43.011626335213137 
 415,19,16,19.104973174542799,19.104973174542799 
 416,19,17,40.607881008493905,40.607881008493905 
 417,19,18,20.880613017821101,20.880613017821101 
 418,19,20,14.317821063276353,14.317821063276353 
 419,19,21,43.104524124504614,43.104524124504614 
 420,20,0,19.026297590440446,19.026297590440446 
 421,20,1,39.408120990476064,39.408120990476064 
 422,20,2,23.086792761230392,23.086792761230392 
 423,20,3,23.706539182259394,23.706539182259394 
 424,20,4,31.016124838541646,31.016124838541646 
 425,20,5,31.256999216175569,31.256999216175569 
 426,20,6,12.041594578792296,12.041594578792296 
 427,20,7,40.112342240263160,40.112342240263160 
 428,20,8,18.973665961010276,18.973665961010276 
 429,20,9,6.324555320336759,6.324555320336759 
 430,20,10,45.254833995939045,45.254833995939045 
 431,20,11,23.409399821439251,23.409399821439251 
 432,20,12,41.677331968349414,41.677331968349414 
 433,20,13,10.770329614269007,10.770329614269007 
 434,20,14,13.000000000000000,13.000000000000000 
 435,20,15,50.960769224963627,50.960769224963627 
 436,20,16,19.026297590440446,19.026297590440446 
 437,20,17,46.389654018972806,46.389654018972806 
 438,20,18,32.202484376209235,32.202484376209235 
 439,20,19,14.317821063276353,14.317821063276353 
 440,20,21,30.149626863362670,30.149626863362670 
 441,21,0,33.120990323358392,33.120990323358392 
 442,21,1,29.832867780352597,29.832867780352597 
 443,21,2,53.235326616824658,53.235326616824658 
 444,21,3,16.643316977093239,16.643316977093239 
 445,21,4,60.638271743182131,60.638271743182131 
 446,21,5,1.414213562373095,1.414213562373095 
 447,21,6,25.059928172283335,25.059928172283335 
 448,21,7,45.803929962395145,45.803929962395145 
 449,21,8,48.093658625644196,48.093658625644196 
 450,21,9,36.013886210738214,36.013886210738214 
 451,21,10,68.447059834590405,68.447059834590405 
 452,21,11,52.239831546435909,52.239831546435909 
 453,21,12,60.745370193949761,60.745370193949761 
 454,21,13,40.607881008493905,40.607881008493905 
 455,21,14,17.262676501632068,17.262676501632068 
 456,21,15,63.655321851358195,63.655321851358195 
 457,21,16,33.120990323358392,33.120990323358392 
 458,21,17,56.080299571239813,56.080299571239813 
 459,21,18,54.129474410897430,54.129474410897430 
 460,21,19,43.104524124504614,43.104524124504614 
 461,21,20,30.149626863362670,30.149626863362670 
 DEPOT_SECTION 
 0 
    """

    solution_text = """
Details of the solution: 
route 0, node_num 29, cost 507.489061, nodes: 0(60.63, 60.63) 18(39.44, 60.63) 10 18(31.99, 51.37) 2 20(0.00, 50.31) 14 3 20(-0.00, 14.32) 19(0.00, 60.63) 11 18(33.16, 60.63) 12 16(22.63, 34.96) 5 21(-0.00, 44.08) 9 19(0.00, 56.27) 8 4 13 20(0.00, 60.63) 6 16(19.73, 42.09) 15 17(0.00, 57.91) 7 1 0(-0.00, -0.00) 
vehicle (route) number: 1 
Total cost: 1507.49
    """
    
    print("Parsing Problem Data...")
    problem_data = parse_evrp_instance(instance_text)
    # print(problem_data)
    print("Problem Data Parsed.\n")
    
    print("Parsing Solution Data...")
    solution_data = parse_solution(solution_text)
    # print(solution_data)
    print("Solution Data Parsed.\n")

    is_feasible, errors_found = validate_solution(problem_data, solution_data)

    print(f"\nFinal Result: Validation {'Passed' if is_feasible else 'Failed'}")
    if errors_found:
        print("Reasons for failure/discrepancies:")
        for err in errors_found:
            print(f"- {err}")