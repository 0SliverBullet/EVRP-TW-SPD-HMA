# include "eval.h"

extern long call_count_move_eval;
extern double mean_duration_move_eval;
extern double mean_route_len;

using namespace std::chrono;
void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, bool &flag, double &cost)
{
    int len = int(nl.size());
    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl)
    {
        load += data.node[node].delivery;
    }
    load += data.node[inserted_node].delivery;

    if (load > capacity)
    {
        flag = false;
        return;
    }

    int pre_node = nl[0];
    bool checked = false;
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        if (i == pos && !checked)
        {
            node = inserted_node;
            i--;
            checked = true;
        }

        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity)
        {
            flag = false;
            return;
        }
        time += data.time[pre_node][node];
        if (time > data.node[node].end)
        {
            flag = false;
            return;
        }
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        distance += data.dist[pre_node][node];
        pre_node = node;
    }

    flag = true;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;
}
void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, int &flag, double &cost)
{   // suppose insert into the pos
    // also check battery constrain
    int len = int(nl.size());
    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl)
    {
        load += data.node[node].delivery;
    }
    load += data.node[inserted_node].delivery;

    if (load > capacity)
    {
        flag = 2;
        return;
    }

    int pre_node = nl[0];
    double arr_remain_dist = data.max_distance_reachable;
    double dep_remain_dist = data.max_distance_reachable;

    bool checked = false;
    for (int i = 1; i < len; i++)
    {    
        //printf("%d\n", i);
        int node = nl[i];
        if (i == pos && !checked)
        {
            node = inserted_node;
            i--;
            checked = true;  //insert once
        }

        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity)
        {
            flag = 2;
            return;
        }
        time += data.time[pre_node][node];

        if (data.node[node].type != 2){   //customer or depot

        if (time > data.node[node].end)
        {
            flag = 3;
            return;
        }
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        arr_remain_dist = dep_remain_dist - data.dist[pre_node][node];
        if (arr_remain_dist < -PRECISION) {
            flag = 4;
            return;
        }
        dep_remain_dist = arr_remain_dist;
        }
        else{
        arr_remain_dist = dep_remain_dist - data.dist[pre_node][node];
        if (arr_remain_dist < -PRECISION) {
            flag = 4;
            return;
        }
        double f_f0_dist=0;
        int j = i;
        bool chk=checked;
        bool if_inserted = false;
        do{
            j++;
            if (j == pos && chk == false) {
               f_f0_dist += data.dist[nl[j-1]][inserted_node];
               j--;
               chk = true;
               if_inserted = true;
            }
            else if (j == pos && chk == true){
               f_f0_dist += data.dist[inserted_node][nl[j]];
               if_inserted = false;
            }
            else{
                f_f0_dist += data.dist[nl[j-1]][nl[j]];
                if_inserted = false;
            }
        } while ((if_inserted == true && data.node[inserted_node].type == 1)||(if_inserted == false && data.node[nl[j]].type == 1)); 
        
        dep_remain_dist = std::max(f_f0_dist, arr_remain_dist);  

        dep_remain_dist = std::min(dep_remain_dist,data.max_distance_reachable);
   
        double max_recharge_time = (dep_remain_dist - arr_remain_dist) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;
        
        double min_remain_time = double(INFINITY);
        double move_time = time + max_recharge_time;
        j = i;
        chk=checked;
        if_inserted = false;
        do{
            j++;
            if (j == pos && chk == false) {
                move_time += data.time[nl[j-1]][inserted_node];

                if (move_time - data.node[inserted_node].start < -PRECISION){
                        double additional_charge_time = std::min(min_remain_time, data.node[inserted_node].start - move_time);
                        max_recharge_time += additional_charge_time;
                        move_time += additional_charge_time;
                        min_remain_time -= additional_charge_time;
                }             
                if (data.node[inserted_node].end - move_time < -PRECISION){
                flag = 3; return;
                }      
                min_remain_time = std::min (min_remain_time, data.node[inserted_node].end-move_time);
                if (min_remain_time == 0) break;
                move_time = std::max(move_time, data.node[inserted_node].start) + data.node[inserted_node].s_time;  
                j--;
                chk = true;
                if_inserted = true;
            }
            else if (j == pos && chk == true){
                move_time += data.time[inserted_node][nl[j]];
                if (move_time - data.node[nl[j]].start < -PRECISION){
                        double additional_charge_time = std::min(min_remain_time, data.node[nl[j]].start - move_time);
                        max_recharge_time += additional_charge_time;
                        move_time += additional_charge_time;
                        min_remain_time -= additional_charge_time;
                }       
                if (data.node[nl[j]].end - move_time < -PRECISION){
                flag = 3; return;
                }            
                min_remain_time = std::min (min_remain_time, data.node[nl[j]].end-move_time);
                if (min_remain_time == 0) break;
                move_time = std::max(move_time, data.node[nl[j]].start) + data.node[nl[j]].s_time;  
                if_inserted = false;
            }
            else{
                move_time += data.time[nl[j-1]][nl[j]];
                if (move_time - data.node[nl[j]].start < -PRECISION){
                        double additional_charge_time = std::min(min_remain_time, data.node[nl[j]].start - move_time);
                        max_recharge_time += additional_charge_time;
                        move_time += additional_charge_time;
                        min_remain_time -= additional_charge_time;
                }            
                if (data.node[nl[j]].end - move_time < -PRECISION){
                    flag = 3; return;
                }       
                min_remain_time = std::min (min_remain_time, data.node[nl[j]].end-move_time);
                if (min_remain_time == 0) break;  //扫描结束时 min_remain_time 可能 > 0
                move_time = std::max(move_time, data.node[nl[j]].start) + data.node[nl[j]].s_time;  
                if_inserted = false;
            }         
        } 
         while ((if_inserted == true && data.node[inserted_node].type == 1)||(if_inserted == false && data.node[nl[j]].type == 1)); 

        dep_remain_dist = std::min(max_recharge_time / data.vehicle.recharging_rate / data.vehicle.consumption_rate + arr_remain_dist, data.max_distance_reachable);

        time += (dep_remain_dist - arr_remain_dist) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;
        }

        distance += data.dist[pre_node][node];
        pre_node = node;
    }

    flag = 1;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;
}

void update_route_status(std::vector<int> &nl, std::vector<status> &sl, Data &data, int &flag, double &cost, int &index_negtive_first)
{
    /*
    flag == 0 route error
    flag == 1 feasible
    flag == 2 capacity violation
    flag == 3 capacity Ok, but time window violation
    flag == 4 capacity & time window Ok, but battery violation only
    */
    int len = int(nl.size());

    // start and end at DC
    if (nl[0] != data.DC || nl[len-1] != data.DC) {flag = 0; return;}

    if (len == 2)
    {
        flag = 1;
        cost = 0.0;
        return;
    }

    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl) {load += data.node[node].delivery;}
    if (load > capacity) {flag = 2; return;}

    int pre_node = nl[0];
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity) {flag = 2; return;}   //capacity constrain violation
        time += data.time[pre_node][node]; 
        sl[i].arr_time = time;

        if (data.node[node].type != 2){  //customer or depot
        if (time > data.node[node].end) {flag = 3; return;}  //time constrain violation
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        sl[i].dep_time = time;

        sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];   
        if (sl[i].arr_RD < -PRECISION) {flag = 4; index_negtive_first = i; return;} 
        sl[i].dep_RD = sl[i].arr_RD;          
        }

        else{  //station
        sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];   
        if (sl[i].arr_RD < -PRECISION) {flag = 4; index_negtive_first = i; return;}
        //update sl[i].dep_RD:
        double f_f0_dist=0;
        int j = i;
        do{
            j++;
            f_f0_dist += data.dist[nl[j-1]][nl[j]];
        } while (data.node[nl[j]].type == 1);

        sl[i].dep_RD = std::max(f_f0_dist,sl[i].arr_RD);  
        
        sl[i].dep_RD = std::min(sl[i].dep_RD,data.max_distance_reachable); 

        double max_recharge_time = (sl[i].dep_RD - sl[i].arr_RD) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;

        double min_remain_time = double(INFINITY);
        double move_time = sl[i].arr_time + max_recharge_time; 
        j = i;
        do{
            j++;
            move_time += data.time[nl[j-1]][nl[j]];
            if (move_time - data.node[nl[j]].start < -PRECISION){
                    double additional_charge_time = std::min(min_remain_time, data.node[nl[j]].start - move_time);
                    max_recharge_time += additional_charge_time;
                    move_time += additional_charge_time;
                    min_remain_time -= additional_charge_time;
            }
            if (data.node[nl[j]].end - move_time < -PRECISION){
                flag = 3; return;
            }             
            min_remain_time = std::min (min_remain_time, data.node[nl[j]].end-move_time);
            if (min_remain_time == 0 ) break;
            move_time = std::max(move_time, data.node[nl[j]].start) + data.node[nl[j]].s_time;
        } while (data.node[nl[j]].type == 1);
        
        sl[i].dep_RD = std::min(max_recharge_time / data.vehicle.recharging_rate / data.vehicle.consumption_rate + sl[i].arr_RD, data.max_distance_reachable);

        time += (sl[i].dep_RD - sl[i].arr_RD) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;

        sl[i].dep_time = time;
        }
        
        distance += data.dist[pre_node][node];  
        pre_node = node;
    }

    flag = 1;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;

}


void update_route_status(bool &evolution, std::vector<int> &nl, std::vector<status> &sl, Data &data, int &flag, double &cost, int &index_negtive_first)
{    
    // auto start = high_resolution_clock::now();
    /*
    flag == 0 route error
    flag == 1 feasible
    flag == 2 capacity violation
    flag == 3 capacity Ok, but time window violation
    flag == 4 capacity & time window Ok, but battery violation only
    */
    /* time complexity O(n) */
    int len = int(nl.size());

    // start and end at DC
    if (nl[0] != data.DC || nl[len-1] != data.DC) {flag = 0; return;}

    if (len == 2)
    {
        flag = 1;
        cost = 0.0;
        return;
    }

    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl) {load += data.node[node].delivery;}
    if (load > capacity) {flag = 2; return;}

    int pre_node = nl[0];
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity) {flag = 2; return;}   //capacity constrain violation


        if (data.node[node].type != 2){  //customer or depot        
        time += data.time[pre_node][node]; 
        sl[i].arr_time = time;
        if (time > data.node[node].end) {flag = 3; return;}  //time constrain violation
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        sl[i].dep_time = time;

        sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];   
        if (sl[i].arr_RD < -PRECISION) {flag = 4; index_negtive_first = i; return;} 
        sl[i].dep_RD = sl[i].arr_RD;          
        }

        else{  //station
        sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];   
        if (sl[i].arr_RD < -PRECISION) {
            //尝试换充电站
            flag = 4;
            for (int k=1;k<data.station_range;k++) {
                  node = data.optimal_staion[pre_node][nl[i+1]][k];
                  sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];
                  if (sl[i].arr_RD > -PRECISION){
                    nl[i]=node;
                    flag = 1;
                    break;
                  }
            }           
            if (flag == 4) {index_negtive_first = i; return; }
            }
        //update sl[i].dep_RD:
        double f_f0_dist=0;
        int j = i;
        do{
            j++;
            f_f0_dist += data.dist[nl[j-1]][nl[j]];
        } while (data.node[nl[j]].type == 1);
  
        sl[i].dep_RD = std::max(f_f0_dist,sl[i].arr_RD);  
        
        if (data.max_distance_reachable - sl[i].dep_RD < -PRECISION) { 
            //尝试换充电站
            flag = 4;
            for (int k=1;k<data.station_range;k++) {
                  node = data.optimal_staion[pre_node][nl[i+1]][k];
                  sl[i].arr_RD = sl[i-1].dep_RD - data.dist[pre_node][node];
                  if (sl[i].arr_RD > -PRECISION){  
                        nl[i]=node;
                        f_f0_dist=0;
                        j = i;
                        do{
                            j++;
                            f_f0_dist += data.dist[nl[j-1]][nl[j]];
                        } while (data.node[nl[j]].type == 1);  
                        sl[i].dep_RD = std::max(f_f0_dist,sl[i].arr_RD); 
                        if (data.max_distance_reachable - sl[i].dep_RD > -PRECISION){
                        flag = 1;
                        break;
                        }
                  }
            }
            if (flag == 4) {index_negtive_first = i; return; }
        }

        //sl[i].dep_RD = std::min(sl[i].dep_RD,data.max_distance_reachable); 
        
        double max_recharge_time = (sl[i].dep_RD - sl[i].arr_RD) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;
        time += data.time[pre_node][node]; 
        sl[i].arr_time = time;

        double min_remain_time = double(INFINITY);
        double move_time = sl[i].arr_time + max_recharge_time; 
        j = i;
        do{
            j++;
            move_time += data.time[nl[j-1]][nl[j]];
            if (move_time - data.node[nl[j]].start < -PRECISION){
                    double additional_charge_time = std::min(min_remain_time, data.node[nl[j]].start - move_time);
                    max_recharge_time += additional_charge_time;
                    move_time += additional_charge_time;
                    min_remain_time -= additional_charge_time;
            }   
            if (data.node[nl[j]].end - move_time < -PRECISION){
                flag = 3; return;
            }                
            min_remain_time = std::min (min_remain_time, data.node[nl[j]].end-move_time);
            if (min_remain_time == 0) break;
            move_time = std::max(move_time, data.node[nl[j]].start) + data.node[nl[j]].s_time;
        } while (data.node[nl[j]].type == 1);
        
        sl[i].dep_RD = std::min(max_recharge_time / data.vehicle.recharging_rate / data.vehicle.consumption_rate + sl[i].arr_RD, data.max_distance_reachable);

        time += (sl[i].dep_RD - sl[i].arr_RD) * data.vehicle.consumption_rate * data.vehicle.recharging_rate;

        sl[i].dep_time = time;
        }
        
        distance += data.dist[pre_node][node];  
        pre_node = node;
    }

    flag = 1;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;

}


void chk_route_O_n(Route &r, Data &data, bool &flag, double &cost)
{
    // auto start = high_resolution_clock::now();

    /* time complexity O(n) */
    std::vector<int> &nl = r.node_list;
    int len = int(nl.size());

    // start and end at DC
    if (nl[0] != data.DC || nl[len-1] != data.DC) {flag = false; return;}
    if (len == 2)
    {
        flag = true;
        cost = 0.0;
        return;
    }

    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl) {load += data.node[node].delivery;}
    if (load > capacity) {flag = false; return;}

    int pre_node = nl[0];
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity) {flag = false; return;}
        time += data.time[pre_node][node];
        if (time > data.node[node].end) {flag = false; return;}
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        distance += data.dist[pre_node][node];
        pre_node = node;
    }

    flag = true;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;
}

bool eval_route(Solution &s, Seq *seqList, int seqListLen, Attr &tmp_attr, Data &data)
{
    const Attr &attr_1 = seqList[0].r_index == -1 ? attr_for_one_node(data, seqList[0].start_point) : s.get(seqList[0].r_index).gat(seqList[0].start_point, seqList[0].end_point);

    const Attr &attr_2 = seqList[1].r_index == -1 ? attr_for_one_node(data, seqList[1].start_point) : s.get(seqList[1].r_index).gat(seqList[1].start_point, seqList[1].end_point);

    if ((!check_tw(attr_1, attr_2, data)) || (!check_capacity(attr_1, attr_2, data)))
        return false;
    connect(attr_1, attr_2, tmp_attr, data.dist[attr_1.e][attr_2.s], data.time[attr_1.e][attr_2.s]);

    for (int i = 2; i < seqListLen; i++)
    {
        const Attr &attr = seqList[i].r_index == -1 ? attr_for_one_node(data, seqList[i].start_point) : s.get(seqList[i].r_index).gat(seqList[i].start_point, seqList[i].end_point);

        if ((!check_tw(tmp_attr, attr, data)) || (!check_capacity(tmp_attr, attr, data)))
            return false;
        connect(tmp_attr, attr, data.dist[tmp_attr.e][attr.s], data.time[tmp_attr.e][attr.s]);
    }
    return true;
}

bool eval_move(Solution &s, Move &m, Data &data, double &base_cost)
{
    std::vector<int> r_indice;
    r_indice.push_back(m.r_indice[0]);
    if (m.r_indice[1] != -2)
        r_indice.push_back(m.r_indice[1]);
    double ori_cost = s.get(r_indice[0]).cal_cost(data);

    if (!data.O_1_evl)  
    {
        // eval the first route
        std::vector<int> target_n_l;
        target_n_l.reserve(MAX_NODE_IN_ROUTE);

        for (int i = 0; i < m.len_1; i++)
        {
            auto &seq = m.seqList_1[i];
            auto &source_n_l = s.get(seq.r_index).node_list;
            for (int index = seq.start_point; index <= seq.end_point; index++)
            {
                target_n_l.push_back(source_n_l[index]);
            }
        }
        Route r(data);
        r.node_list = target_n_l;
        bool flag = false;
        double new_cost = 0.0;
        chk_route_O_n(r, data, flag, new_cost);
        if (!flag) return false;

        // eval the second route
        if (int(r_indice.size()) == 2)
        {
            std::vector<int> target_n_l;
            target_n_l.reserve(MAX_NODE_IN_ROUTE);

            for (int i = 0; i < m.len_2; i++)
            {
                auto &seq = m.seqList_2[i];
                if (seq.r_index == -1)
                {
                    target_n_l.push_back(data.DC);
                    continue;
                }
                auto &source_n_l = s.get(seq.r_index).node_list;
                for (int index = seq.start_point; index <= seq.end_point; index++)
                {
                    target_n_l.push_back(source_n_l[index]);
                }
            }
            Route r(data);
            r.node_list = target_n_l;
            if (r_indice[1] != -1)
                ori_cost += s.get(r_indice[1]).cal_cost(data);
            bool flag = false;
            double cost = 0.0;
            chk_route_O_n(r, data, flag, cost);
            if (!flag) return false;
            new_cost += cost;
        }
        m.delta_cost = new_cost - ori_cost;
        return true;
    }

    /* eval the connection of the seqeunces in m */
    // auto start = high_resolution_clock::now();

    Attr tmp_attr_1;
    if (!eval_route(s, m.seqList_1, m.len_1, tmp_attr_1, data))
        return false;
    double new_cost = 0.0;
    if (tmp_attr_1.num_cus != 0)
        new_cost += data.vehicle.d_cost + tmp_attr_1.dist * data.vehicle.unit_cost;
    if (int(r_indice.size()) == 2)
    {
        Attr tmp_attr_2;
        if (!eval_route(s, m.seqList_2, m.len_2, tmp_attr_2, data))
            return false;
        if (r_indice[1] != -1)
            ori_cost += s.get(r_indice[1]).cal_cost(data);
        if (tmp_attr_2.num_cus != 0)
            new_cost += data.vehicle.d_cost + tmp_attr_2.dist * data.vehicle.unit_cost;
    }
    m.delta_cost = new_cost - ori_cost;

    if (m.delta_cost < -PRECISION) {

        if (base_cost == -1) return true; //use Aggressive Local Search (ALS) 

        //else use Conservative Local Search
           
        Solution item = s;
        std::vector<int> tour_id_array;
        //clock_t stime1 = clock();
        tour_id_array = apply_move(item, m, data);
        //double used_sec1 = (clock() - stime1) / (CLOCKS_PER_SEC*1.0);
        //printf("time: %.10lf sec\n", used_sec1);
        double ori_cost = s.get(m.r_indice[0]).total_cost;
        if (m.r_indice[1] >= 0) ori_cost += s.get(m.r_indice[1]).total_cost;
        double new_cost = 0.0;
        for (auto j: tour_id_array) {
                if (j >= item.len()) {
                    if (m.r_indice[0] < item.len() && m.r_indice[1] < item.len()) ori_cost += s.get(j).total_cost;
                    continue;
                }
                Route r= item.get(j);
                std::swap(r.node_list, r.customer_list);
                r.temp_node_list = r.customer_list;
                int flag = 0;
                double cost = 0.0;
                int index_negtive_first = -1;
                update_route_status(r.temp_node_list,r.status_list,data,flag,cost,index_negtive_first); 
                if (flag == 0 || flag == 2 || flag == 3) return false;
                if (flag == 1) { item.get(j).total_cost = item.get(j).cal_cost(data); }
                if (flag == 4 && ! parallel_sequential_station_insertion(item, r, data, j)) return false;
                new_cost += item.get(j).total_cost;
        } 

        m.delta_cost = new_cost - ori_cost;
        // printf("2: %.2lf \n", m.delta_cost);
        if (m.delta_cost > -PRECISION){
            return false;
        }
        for (int j = 0; j < tour_id_array.size(); j++){
                if (tour_id_array[j] >= item.len()) continue;
                m.list[j] = item.get(tour_id_array[j]).node_list;
                m.total_cost[j] = item.get(tour_id_array[j]).total_cost;
        }        
        return true;
    }
    return false;
}

bool parallel_sequential_station_insertion(Solution &item, Route &r, Data &data, int &j){
    double evolution_cost=double(INFINITY);
    double heuristic_cost=double(INFINITY);
    int dimension=r.customer_list.size()-1;
    //clock_t stime1 = clock();
    if (data.parallel_insertion) parallel_station_insertion(dimension,r,data,evolution_cost);
    //double used_sec1 = (clock() - stime1) / (CLOCKS_PER_SEC*1.0);
    item.get(j).node_list = r.node_list;
    r.temp_node_list=r.customer_list;
    int flag = 0;
    double new_cost = 0.0;
    int index_negtive_first = -1;
    update_route_status(r.temp_node_list,r.status_list,data,flag,new_cost,index_negtive_first); 
    std::vector<std::pair<int,int>> station_insert_pos;
    station_insert_pos.clear();            
    //clock_t stime2 = clock();
    //double used_sec2 = 0.0;
    if (sequential_station_insertion(flag, index_negtive_first, r, data, station_insert_pos, heuristic_cost)){
        
        for (int i=0; i<station_insert_pos.size(); i++){
                    r.customer_list.insert(r.customer_list.begin()+ station_insert_pos[i].second, station_insert_pos[i].first); 
        }
        if (r.customer_list.size() >= 4) sequential_station_improvement(heuristic_cost, data, r);
     
    }
    // used_sec2 = (clock() - stime2) / (CLOCKS_PER_SEC*1.0);
    // printf("evolution: %.2lf in %lf sec, heuristic:  %.2lf in %lf sec\n", \
    //              evolution_cost, used_sec1, heuristic_cost, used_sec2);
    if (evolution_cost == double(INFINITY) && heuristic_cost == double(INFINITY)) {
                    item.cost=double(INFINITY);
                    return false;
    }
    else {
        if (heuristic_cost - evolution_cost < -PRECISION){
            item.get(j).node_list=r.customer_list; 
        }                    
    }
    item.get(j).update(data);
    item.get(j).total_cost = item.get(j).cal_cost(data);
    return true;
}

void sequential_station_improvement(double &cost, Data &data, Route &r){
    int flag = 0, j, k, check = 0;
    double new_cost = 0.0, previous_cost = cost;
    int index_negtive_first = -1;
    std::vector<int> n_l = r.customer_list;
    int first_node, second_node, third_node;
    for (j = 0; j < r.customer_list.size()-2; j++){
            first_node = r.customer_list[j];
            second_node = r.customer_list[j+1];
            third_node = r.customer_list[j+2];
            if (data.node[second_node].type == 2){   
            //improve r by adjusting the station fk in (c, fk, fj ), (fi, fk, c), (fi, fk, fj ) pattern;
                    if (data.node[first_node].type ==2 || data.node[third_node].type ==2 ){  
                            for (k=0; k<data.station_range;k++){
                                //printf("%d, %d -> %d, %d\n",first_node,second_node,data.optimal_staion[first_node][third_node][k],third_node);
                                if (data.optimal_staion[first_node][third_node][k] == second_node) break;
                                        n_l[j+1] = data.optimal_staion[first_node][third_node][k];
                                        r.temp_node_list=n_l;
                                        flag = 0;
                                        new_cost = 0.0;
                                        index_negtive_first = -1;
                                        update_route_status(r.temp_node_list,r.status_list,data,flag,new_cost,index_negtive_first);
                                        if (flag == 0|| flag==2 || flag == 3) continue;
                                        if (flag == 4) { 
                                        double heuristic_cost=double(INFINITY);
                                        std::vector<std::pair<int,int>> station_insert_pos;
                                        station_insert_pos.clear();            
                                        if (sequential_station_insertion(flag, index_negtive_first, r, data, station_insert_pos, heuristic_cost)){
                                                for (int index=0; index<station_insert_pos.size(); index++){
                                                    n_l.insert(n_l.begin()+ station_insert_pos[index].second, station_insert_pos[index].first); 
                                                }  
                                                new_cost = heuristic_cost;
                                                          
                                        }
                                        else continue; 
                                        }
                                        
                                        if (new_cost-previous_cost<-PRECISION)  {
                                            //printf("%d, %d -> %d, %d: %.2lf\n",first_node,second_node,data.optimal_staion[first_node][third_node][k],third_node, new_cost-pre_cost);
                                            cost = new_cost;
                                            r.customer_list = n_l;
                                            check = 1;
                                            break;
                                        }    
                                        else{
                                            n_l=r.customer_list;
                                        }                                                   
                                }

                                if (check == 1){
                                    break;
                                }
                    }
            }
    }
}

std::vector<int> apply_move(Solution &s, Move &m, Data data)
{
    std::vector<int> r_indice;
    r_indice.push_back(m.r_indice[0]);
    if (m.r_indice[1] != -2)
        r_indice.push_back(m.r_indice[1]);

    // handle the first route
    if (r_indice[0] == -2 || r_indice[0] == -1)
    {
        printf("Error: detect -1 or -2 in r_indice[0] in move\n");
        exit(-1);
    }
    Route &r = s.get(r_indice[0]);
    std::vector<int> target_n_l;
    target_n_l.reserve(MAX_NODE_IN_ROUTE);

    for (int i = 0; i < m.len_1; i++)
    {
        auto &seq = m.seqList_1[i];
        auto &source_n_l = s.get(seq.r_index).node_list;
        if (seq.start_point <= seq.end_point)
        {
            for (int index = seq.start_point; index <= seq.end_point; index++)
                {target_n_l.push_back(source_n_l[index]);}
        }
        else
        {
            for (int index = seq.start_point; index >= seq.end_point; index--)
            {
                target_n_l.push_back(source_n_l[index]);
            }
        }
    }

    // handle the second route
    if (int(r_indice.size()) == 2)
    {
        std::vector<int> target_n_l_2;
        target_n_l_2.reserve(MAX_NODE_IN_ROUTE);

        for (int i = 0; i < m.len_2; i++)
        {
            auto &seq = m.seqList_2[i];
            if (seq.r_index == -1)
            {
                target_n_l_2.push_back(data.DC);
                continue;
            }
            auto &source_n_l = s.get(seq.r_index).node_list;
            if (seq.start_point <= seq.end_point)
            {
                for (int index = seq.start_point; index <= seq.end_point; index++)
                {
                    target_n_l_2.push_back(source_n_l[index]);
                }
            }
            else
            {
                for (int index = seq.start_point; index >= seq.end_point; index--)
                {
                    target_n_l_2.push_back(source_n_l[index]);
                }
            }
        }
        if (r_indice[1] == -1)
        {
            Route r(data);
            r.node_list = target_n_l_2;
            r.update(data);
            s.append(r);
            r_indice[1] = s.len() - 1;
        }
        else
        {
            Route &r = s.get(r_indice[1]);
            r.node_list = target_n_l_2;
            r.update(data);
        }
    }
    // set node list only at the end
    r.node_list = target_n_l;
    r.update(data);

    s.local_update(r_indice);
    return r_indice;
}

bool sequential_station_insertion(int &flag, int &index_negtive_first, Route &r, Data &data, std::vector<std::pair<int,int>> &station_insert_pos,double &heuristic_cost){
    std::vector<double> score(MAX_STATION_POINT);
    std::vector<int> score_argrank(MAX_STATION_POINT);
    std::vector<int> ties(MAX_STATION_POINT);
    std::vector<int> feasible_pos(MAX_NODE_IN_ROUTE*MAX_POINT, 0);
    int station_pos_num = 0;
    bool station_pos_type = false;  
    double new_cost = 0.0;
    while (flag == 4){
            int index_last_f0 = index_negtive_first;
            do {
              index_last_f0--;
            }while (data.node[r.temp_node_list[index_last_f0]].type == 1);
            // -----------------------------------
            int path_len = index_negtive_first - index_last_f0;
            if (data.station_num <= path_len){
                station_pos_num = data.station_num;
                station_pos_type = false; 
            }
            else{
                station_pos_num = path_len;
                station_pos_type = true;
            }
            std::vector<int> station_pos(station_pos_num);
            // ----------------------------------
            if (cal_score_station(station_pos_type, feasible_pos,station_pos,score,r,data,index_last_f0,index_negtive_first))
            {
            argsort(score, score_argrank, station_pos_num);
            double best_score = score[score_argrank[0]];
            ties[0] = score_argrank[0];

            int selected;
            int i = 1;
            for (; i < station_pos_num; i++)
            {
                if (std::abs(best_score - score[score_argrank[i]]) < -PRECISION)
                    ties[i] = score_argrank[i];
                else
                    break;
            } //consider the same score
            if (i > 1) selected = ties[randint(0, i - 1, data.rng)];
            else selected = ties[0];
            
            int node; 
            int pos;
            if (station_pos_type == false){
                    node = selected + data.customer_num+1;
                    pos = station_pos[selected];                
            }
            else{
                    pos = selected + index_last_f0 + 1;
                    node = station_pos[selected];               
            }

            if (r.temp_node_list[pos-1]==node || r.temp_node_list[pos] == node \
            || (pos - 2 >=0 && r.temp_node_list[pos-2]==node && data.node[r.temp_node_list[pos-1]].type == 2) \
            || (pos + 1 <= r.temp_node_list.size()-1 && r.temp_node_list[pos+1]==node && data.node[r.temp_node_list[pos]].type == 2)) {
                break;    
            }
            station_insert_pos.push_back({node,pos});
            r.temp_node_list.insert(r.temp_node_list.begin() + pos, node);
            flag = 0;
            new_cost = 0.0;
            index_negtive_first = -1;
            update_route_status(r.temp_node_list, r.status_list, data,flag,new_cost,index_negtive_first);
            heuristic_cost = new_cost;
            }
            else{ 
            break;
            }
        }
        
        if (flag==1) {
        return true;
        }
        else{
        heuristic_cost = double(INFINITY);
        return false;
        }
}

bool cal_score_station(bool type, std::vector<int> &feasible_pos, std::vector<int> &station_pos, std::vector<double> &score, Route &r,Data &data,int index_last_f0, int index_negtive_first)
{
    if (index_negtive_first == -1) return false;
    int r_len = int(r.temp_node_list.size());
    // filter all infeasible positions
    int count1 = 0, count4 = 0, relax = 1;          
    for (int pos=index_last_f0+1; pos <= index_negtive_first; pos++){
            for (int j = 0; j <data.station_range; j++){
            int i = data.optimal_staion[r.temp_node_list[pos-1]][r.temp_node_list[pos]][j];
            int flag = 0;
            double cost = -1.0;
            if (r.status_list[pos-1].dep_RD - data.dist[r.temp_node_list[pos-1]][i] < -PRECISION \
                || (pos == 1 && data.dist[r.temp_node_list[pos-1]][i] == 0)\
                || i == r.temp_node_list[pos-1] \
                || i == r.temp_node_list[pos] \
                || (data.node[r.temp_node_list[pos]].type == 0 && data.dist[i][r.temp_node_list[pos]] == 0)) {
                flag = 0;
            }
            else chk_nl_node_pos_O_n(r.temp_node_list, i, pos, data, flag, cost); 
            if (flag == 1) {
                feasible_pos[i*MAX_NODE_IN_ROUTE+pos] = 1;
                count1++;
            }else if (flag ==4){
                feasible_pos[i*MAX_NODE_IN_ROUTE+pos] = 4;
                count4++;
            }else {
                feasible_pos[i*MAX_NODE_IN_ROUTE+pos] = 0;
            }            
          }
    }
    if (count1 + count4 ==0) return false;
    if (count1 == 0) relax = 4;
    if (type ==false) {
            for (int i = data.customer_num+1; i <= data.customer_num + data.station_num; i++)
            {
                double best_score = double(INFINITY);
                int best_pos = -1;
                for (int pos=index_last_f0+1; pos <= index_negtive_first; pos++)
                {
                    if (feasible_pos[i*MAX_NODE_IN_ROUTE+pos] != relax) continue;
                    //if (feasible_pos[i*MAX_NODE_IN_ROUTE+pos] == 0) continue;
                    double utility = criterion_station(r, data, i, pos);
                    if (utility - best_score < -PRECISION)
                    {
                        best_score = utility;
                        best_pos = pos;
                    }
                }
                station_pos[i-data.customer_num-1] = best_pos;
                score[i-data.customer_num-1] = best_score;
            }        
    }
    else{
            for (int pos=index_last_f0+1; pos <= index_negtive_first; pos++)
            {
                double best_score = double(INFINITY);
                int best_station = -1;
                for (int j = 0; j <data.station_range; j++)
                {
                    int i = data.optimal_staion[r.temp_node_list[pos-1]][r.temp_node_list[pos]][j];
                    if (feasible_pos[i*MAX_NODE_IN_ROUTE+pos]!=relax) continue;
                    //if (feasible_pos[i*MAX_NODE_IN_ROUTE+pos] == 0) continue;
                    double utility = criterion_station(r, data, i, pos);
                    if (utility - best_score < -PRECISION)
                    {
                        best_score = utility;
                        best_station = i;
                    }
                }
                station_pos[pos-index_last_f0-1] = best_station;
                score[pos-index_last_f0-1] = best_score;
            } 
    }

    return true;    

}

double criterion_station(Route &r, Data &data, int node, int pos)
{
    std::vector<int> &nl = r.temp_node_list;
    double alpha=1.0, beta=1.0;
    // TD
    int pre = nl[pos-1];
    int suc = nl[pos];
    double td = data.dist[pre][node] + data.dist[node][suc] - data.dist[pre][suc];
    
    return td;
}