# EVRP-TW-SPD: HMA, Benchmark 



The HMA source code and datasets used in our paper "Hybrid Memetic Search for Electric Vehicle Routing with Time Windows, Simultaneous Pickup-Delivery, and Partial Recharges".





## File Structure

```
EVRP-TW-SPD-HMA-code-dataset/
│
├── README.md                   # overview and setup instructions
│
├── src/                        # HMA source code 
│   └── evrp_tw_spd_solver.cpp  # main function 
│   └── evrp_tw_spd_solver.h   
│   └── ...  
│
├── data/                       # datasets used in our experiments
│   ├── akb_instances           # the akb set
│   └── jd_instances            # the jd set (the new benchmark set)
│
├── solution/                   # solutions obtained in 10 independent runs
│   ├── akb
│   ├── jd/
|   |   ├── small_timelimit
|   |   ├── large_timelimit
|   |   └── ... 
|   |
│   └── ... 
|
└── .gitattributes        
```





## Usage

To repeat our experiments in Linux, if current directory is `EVRP-TW-SPD-HMA-code-dataset`, then run the following commands:

```bash
mkdir bin
cd src
```

**compile:**

```bash
g++ -std=c++11 -o ../bin/evrp-tw-spd -O3 evrp_tw_spd_solver.cpp eval.cpp operator.cpp search_framework.cpp solution.cpp util.cpp data.cpp evolution.cpp
```

**execute:**

```bash
cd ..
./bin/evrp-tw-spd [--problem PROBLEM] [--pruning] [--output OUTPUT] [--time TIME] [--runs RUNS] [--g_1 G_1] [--pop_size POP_SIZE] [--init INIT] [--cross_repair CROSS_REPAIR] [--parent_selection PARENT_SELECTION] [--replacement REPLACEMENT] [--O_1_eval] [--two_opt] [--two_opt_star] [--or_opt OR_OPT] [--two_exchange TWO_EXCHANGE] [--elo ELO] [--related_removal] [--removal_lower REMOVAL_LOWER] [--removal_upper REMOVAL_UPPER] [--regret_insertion] [--individual_search LNS] [--population_search MS] [--parallel_insertion PSSI] [--conservative_local_search CLS] [--aggressive_local_search ALS] [--station_range sr] [--subproblem_range K_SUBPROBLEM]
```

1. on small-scale instances with 5/10/15 customers, 2~8 stations:

```bash
./bin/evrp-tw-spd --problem ./data/akb_instances/c101c5.txt --pruning --time 105 --runs 10 --g_1 20 --pop_size 9 --init rcrs --cross_repair regret --parent_selection circle --replacement one_on_one --O_1_eval --two_opt --two_opt_star --or_opt 2 --two_exchange 2 --elo 1 --related_removal --removal_lower 0.2 --removal_upper 0.4 --regret_insertion --individual_search --population_search --parallel_insertion --conservative_local_search --aggressive_local_search --station_range 1.0 --subproblem_range 1
```

2. on medium-scale instances with 100 customers, 21 stations:

```bash
./bin/evrp-tw-spd --problem ./data/akb_instances/c101_21.txt --pruning --time 630 --runs 10 --g_1 20 --pop_size 4 --init rcrs --cross_repair regret --parent_selection circle --replacement one_on_one --O_1_eval --two_opt --two_opt_star --or_opt 2 --two_exchange 2 --elo 1 --related_removal --removal_lower 0.1 --removal_upper 0.2 --regret_insertion --individual_search --population_search --parallel_insertion --conservative_local_search --aggressive_local_search --station_range 0.5 --subproblem_range 1
```

3. on large-scale instances with 200/400/600/800/1000 customers, 100 stations `--subproblem_range 2/4/6/8/10`:

```bash
./bin/evrp-tw-spd --problem ./data/jd_instances/jd200_1.txt --pruning --time 1800 --runs 10 --g_1 20 --pop_size 4 --init rcrs --cross_repair regret --parent_selection circle --replacement one_on_one --O_1_eval --two_opt --two_opt_star --or_opt 2 --two_exchange 2 --elo 1 --related_removal --removal_lower 0.05 --removal_upper 0.05 --regret_insertion --individual_search --population_search --parallel_insertion --aggressive_local_search --station_range 0.1 --subproblem_range 2
```





## Benchmark

The benchmark contains both the *akb* set and the *jd* set.

**Instance File Structure  (the corresponding variables in paper follow "i.e.,")**

1. Basic Information: This section contains detailed information about a electric vehicle.

| Item                 | Comment                                                      |
| -------------------- | ------------------------------------------------------------ |
| **NAME**             | The name of the instance, e.g.,  `c101_21`, `200_1`.         |
| **TYPE**             | Specifies the problem type, which is `EVRP-TW-SPD`.          |
| **DIMENSION**        | The total number of nodes in the instance, i.e., $1+M+P$.    |
| **VEHICLES**         | Ignore.                                                      |
| **DISPATCHINGCOST**  | The cost incurred when dispatching a vehicle, i.e., $\mu_1$. |
| **UNITCOST**         | The cost per unit distance, i.e., $\mu_2$.                   |
| **CAPACITY**         | The maximum loading capacity of each vehicle, i.e, $C$.      |
| **ELECTRIC_POWER**   | The maximum battery capacity of each vehicle, i.e., $Q$.     |
| **CONSUMPTION_RATE** | The the EV’s battery energy consumption rate per unit of travel distance, i.e., $h$. |
| **RECHARGING_RATE**  | The recharging rate, i.e., $g$. Note that it takes time $g$ to charge one unit of battery energy. |
| **EDGE_WEIGHT_TYPE** | Ignore.                                                      |

2. Node Section (NODE_SECTION): This section contains detailed information about each node in the instance.

| Item                              | Comment                                                      |
| --------------------------------- | ------------------------------------------------------------ |
| **ID**                            | Unique identifier for the node, i.e., $i \in \boldsymbol{V}'$. |
| **type**                          | The type of node (`d` for depot, `c` for customer, `f` for charging station). |
| **x / lng**                       | The x-coordinate (in the *akb* set) / longitude (in the *jd* set) of the node, i.e., $x_i / lng_i$. |
| **y / lat**                       | The y-coordinate (in the *akb* set) / latitude (in the *jd* set)  of the node, i.e., $y_i / lat_i$. |
| **delivery**                      | The amount of goods delivered to the node, i.e., $u_i$.      |
| **pickup**                        | The amount of goods to be picked up at the node, i.e., $v_i$. |
| **ready_time / first_receive_tm** | The earliest time at which service at the node can begin. i.e., $e_i$. |
| **due_date /  last_receive_tm**   | The latest time at which service at the node can begin. i.e., $l_i$. |
| **service_time**                  | The time required to service the node, i.e., $s_i$.          |

3. Distance and Time Section (DISTANCETIME_SECTION): This section provides the distances and time taken to travel between nodes. 

| Item          | Comment                                                      |
| ------------- | ------------------------------------------------------------ |
| **ID**        | Unique identifier for the directly connected edge, i.e., $(i, j) \in \boldsymbol{E}$. |
| **from_node** | The starting node ID, i.e., $i$.                             |
| **to_node**   | The ending node ID, i.e, $j$.                                |
| **distance**  | The travel distance between the two nodes, i.e., $d_{ij}$.   |
| **spend_tm**  | The travel time between the two nodes, i.e., $t_{ij}$.       |

4. Depot Section (DEPOT_SECTION): This section indicates the depot's ID, i.e., $0$.
