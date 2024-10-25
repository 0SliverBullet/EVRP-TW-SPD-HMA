#include "search_framework.h"
extern clock_t find_best_time;
extern clock_t find_bks_time;
extern int find_best_run;
extern int find_best_gen;
extern int find_bks_run;
extern int find_bks_gen;
extern bool find_better;
extern long call_count_move_eval;
extern long mean_duration_move_eval;

void update_best_solution(Solution &s, Solution &best_s, clock_t used, int run, int gen, Data &data, int level,  clock_t stime0, double &update_value)
{
    if (s.cost - best_s.cost < -PRECISION)
    {
        double delta = s.cost - best_s.cost;
        best_s = s;

        if (level == 1) {
            printf("Best solution update: %.4f\n", best_s.cost);        

        }
        else  {
            update_value += delta;
            clock_t used0 = (clock() - stime0) / CLOCKS_PER_SEC;
            printf("Best solution update: %.4f\n", update_value);
 
        }

        find_best_time = used;
        find_best_run = run;
        find_best_gen = gen;
        if (!find_better && (std::abs(best_s.cost - data.bks) < PRECISION ||
                             (best_s.cost - data.bks < -PRECISION)))
        {
            find_better = true;
            find_bks_time = used;
            find_bks_run = run;
            find_bks_gen = gen;
        }
    }


}
void decomposition_cluster(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data, clock_t stime){
    clock_t used = 0; 
    int sub_problem_num = data.subproblem_range;
    printf("sub_problem_num: %d\n", sub_problem_num);
    int Max_customer_num_per_subproblem = data.customer_num / sub_problem_num + (data.customer_num % sub_problem_num != 0);
    int len = pop.size();
    for (int index = 0; index < len; index++){
        int i = pop_argrank[index];
        std::vector<std::vector<int>> subproblem(sub_problem_num);
        for (int j = 0; j< pop[i].len(); j++ ){
            Route &r = pop[i].get(j);
            r.customer_list.clear();
            for (int k=0; k<r.node_list.size(); k++){
                    if (data.node[r.node_list[k]].type !=2 ){
                            r.customer_list.push_back(r.node_list[k]);
                        }
            }
            r.node_num = r.node_list.size() - 1; // - 1 depot 
            r.customer_num = r.customer_list.size() - 2;  // - 2 depots
            // calculate Centroids
            r.cluster = -1;
            r.x = 0;
            r.y = 0;
            for (int k=0; k<r.node_list.size()-1; k++){
                    r.x += data.node[r.node_list[k]].x;
                    r.y += data.node[r.node_list[k]].y;
            } 
            r.x = r.x * 1.0 / r.node_num;
            r.y = r.y * 1.0 / r.node_num;
            }
        std::vector<int> clusterSize(sub_problem_num, 0);
        pop[i].balancedCluster(sub_problem_num, Max_customer_num_per_subproblem, clusterSize);
        // assemble subproblems
        std::vector<Solution> s_d(sub_problem_num);
        std::vector<double> s_d_fit(sub_problem_num);
        for (int j = 0; j< pop[i].len(); j++ ){
            Route &r = pop[i].get(j);
            s_d[r.cluster].append(r);
            std::vector<int> &nl = r.customer_list;
            int customer_num_in_route = nl.size()-2;
            auto start = nl.begin() + 1;  
            auto end = nl.begin() + customer_num_in_route + 1;   
            subproblem[r.cluster].insert(subproblem[r.cluster].end(), start, end);  
        }
        Solution s_m;
        s_m.cost = 0;
        double update_cost = pop[i].cost;
        for (int h = 0; h < sub_problem_num; ++h) { 
            if (clusterSize[h] > 0){
                s_d[h].cal_cost(data);
                s_d_fit[h] = s_d[h].cost;
                printf("subproblem %d, customer: %d\n", h+1, clusterSize[h]);
                Solution best_s = s_d[h];
                // printf("%.2lf ", best_s.cost);
                // best_s.cost = double(INFINITY);
                Data sub_data(data, subproblem[h]);
                std::map<int, int> mapping;
                // master_to_sub
                mapping.insert(std::make_pair(0, 0));
                for (int k = 0; k< subproblem[h].size(); k++){
                    mapping.insert(std::make_pair(subproblem[h][k], k + 1));
                }
                for (int k = sub_data.customer_num+1; k <= sub_data.customer_num +sub_data.station_num; k++){
                    mapping.insert(std::make_pair(data.customer_num+k-sub_data.customer_num, k));
                }

                for (int k= 0; k< best_s.len(); k++){
                    Route &r = best_s.get(k);
                    for (int j = 0; j < r.node_list.size(); j++){
                        int node = r.node_list[j];
                        int master_to_sub = mapping[node];
                        r.node_list[j] = master_to_sub;
                    }
                    for (int j = 0; j < r.customer_list.size(); j++){
                        int node = r.customer_list[j];
                        int master_to_sub = mapping[node];
                        r.customer_list[j] = master_to_sub;
                    }
                    r.update(sub_data);
                    r.total_cost = r.cal_cost(sub_data);
                }
                best_s.cal_cost(sub_data);
                // printf("%.2lf\n", best_s.cost);
                if (data.tmax != NO_LIMIT && (clock() - stime) / (CLOCKS_PER_SEC*1.0) > clock_t(data.tmax))
                {                
                    // time_exhausted = true;
                    // break;
                }
                else search_framework(sub_data, best_s, 0, stime, update_cost);

                if (best_s.cost -  s_d_fit[h] < -PRECISION) { //subproblem is optimized

                        update_cost += best_s.cost -  s_d_fit[h];
                        used = (clock() - stime) / CLOCKS_PER_SEC;

                        s_d_fit[h] = best_s.cost;
                        //node number restoration
                        Solution s_t;
                        for (int k = 0; k < best_s.len(); k++){
                            Route r(data);
                            r.node_list.clear();
                            r.customer_list.clear();
                            for (int node: best_s.get(k).node_list){ 
                                int sub_to_master = 0;
                                if (sub_data.node[node].type == 0){
                                }else if (sub_data.node[node].type ==1){
                                    sub_to_master = subproblem[h][node-1];
                                }else if (sub_data.node[node].type == 2){
                                    sub_to_master = data.customer_num + node - sub_data.customer_num;
                                } 
                                r.node_list.push_back(sub_to_master);
                                if (data.node[sub_to_master].type !=2){
                                    r.customer_list.push_back(sub_to_master);
                                }      
                            }
                           
                            r.update(data);
                            r.total_cost = r.cal_cost(data);
                            s_t.append(r);
                        }
                        s_t.cal_cost(data);
                        printf("%.2lf, %.2lf, %.2lf\n", s_d[h].cost, s_t.cost, best_s.cost);
                        s_d[h] =s_t;           
                }
             

                for (int j = 0; j< s_d[h].len(); j++ ){
                    Route &r = s_d[h].get(j);
                    s_m.append(r);
                }    
                s_m.cost += s_d_fit[h];                         

            }
        }

        pop[i] = s_m;
        pop_fit[i] = pop[i].cost;

    }   
}

void initialization(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data)
{
    int len = int(pop.size());
    printf("Initialization, using %s method\n", data.init.c_str());
    if (data.init == RCRS)
    {
        for (int i = 0; i < len; i++)
        {
            pop[i].clear(data);
        }
        data.n_insert = RCRS;
        data.ksize = data.k_init;
        for (int i = 0; i < len; i++)
        {
            // printf("individual %d:\n",i);
            data.lambda_gamma = data.latin[i];
            if (len == 1) data.lambda_gamma = std::make_tuple(0.5, 0.5);
            // printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
            new_route_insertion(pop[i], data);
        }
    }
    else if (data.init == RCRS_RANDOM)
    {
        for (int i = 0; i < len; i++)
        {
            pop[i].clear(data);
        }
        data.n_insert = RCRS;
        data.ksize = data.k_init;
        for (int i = 0; i < len; i++)
        {
            data.lambda_gamma = std::make_tuple(rand(0, 1, data.rng), rand(0, 1, data.rng));
            printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
            new_route_insertion(pop[i], data);
        }
    }
    else if (data.init == TD)
    {
        for (int i = 0; i < len; i++)
        {
            pop[i].clear(data);
        }
        data.ksize = data.k_init;
        data.n_insert = TD;
        for (int i = 0; i < len; i++) {new_route_insertion(pop[i], data);}
    }
    else if (data.init == PERTURB)
    {
        for (int i = 1; i < len; i++)
        {
            pop[i].clear(data);
        }
        data.ksize = data.k_init;
        data.n_insert = RCRS;
        std::vector<Solution> s_vector(1);
        double w1 = data.destroy_ratio_l, w2 = data.destroy_ratio_u;
        for (int i = 1; i < len; i++){
            if (i % 2 == 1){
                    double w = 1.0 * i / len;
                    data.destroy_ratio_l = w;
                    data.destroy_ratio_u = w;                   
                    s_vector[0] = pop[0];
                    perturb(s_vector, data); 
                    pop[i] = s_vector[0];
            }
            else{
                    data.lambda_gamma = data.latin[i];
                    new_route_insertion(pop[i], data);               
            }
        }
        shuffle(pop.begin(), pop.end(), data.rng);
        data.destroy_ratio_l = w1;
        data.destroy_ratio_u = w2;
    }
    else
    {
        /* more insertion heuristic */
    }

    for (int i = 0; i < len; i++)
    {
        pop_fit[i] = pop[i].cost; 
        printf("Solution %d, cost %.2f, m %d\n", i, pop_fit[i], pop[i].len());
    }

    argsort(pop_fit, pop_argrank, len);

    printf("best cost %.2f, m %d\n", pop_fit[pop_argrank[0]], pop[pop_argrank[0]].len());
    printf("Initialization done.\n");
    
}

void tournament(vector<int> &indice, vector<double> pop_fit, int boundray, Data &data)
{
    int index_index_1 = randint(0, boundray, data.rng);
    // swap two values
    int tmp = indice[index_index_1];
    indice[index_index_1] = indice[boundray];
    indice[boundray] = tmp;
    int index_index_2 = randint(0, boundray-1, data.rng);
    tmp = indice[index_index_2];
    indice[index_index_2] = indice[boundray-1];
    indice[boundray-1] = tmp;
    // select one value from indice[boundray-1] and indice[boundray]
    int selected;
    if (abs(pop_fit[indice[boundray]] - pop_fit[indice[boundray-1]]) < PRECISION)
        selected = randint(boundray-1, boundray, data.rng);
    else if (pop_fit[indice[boundray]] < pop_fit[indice[boundray-1]])
        selected = boundray;
    else
        selected = boundray - 1;
    tmp = indice[selected];
    indice[selected] = indice[boundray];
    indice[boundray] = tmp;
}

void select_parents(vector<Solution> &pop, vector<double> pop_fit, vector<tuple<int, int>> &p_indice, Data &data)
{
    int len = data.p_size;
    vector<int> indice(len);
    iota(indice.begin(), indice.end(), 0);
    shuffle(indice.begin(), indice.end(), data.rng);
    if (data.selection == CIRCLE)
    {
        for (int i = 0; i < len - 1; i++)
        {
            get<0>(p_indice[i]) = indice[i];
            get<1>(p_indice[i]) = indice[i+1];
        }
        get<0>(p_indice[len-1]) = indice[len-1];
        get<1>(p_indice[len-1]) = indice[0];
    }
    else if (data.selection == TOURNAMENT)
    {
        for (int i = 0; i < len; i++)
        {
            tournament(indice, pop_fit, len-1, data);
            tournament(indice, pop_fit, len-2, data);
            get<0>(p_indice[i]) = indice[len-1];
            get<1>(p_indice[i]) = indice[len-2];
        }
    }
    else if (data.selection == RDSELECTION)
    {
        for (int i = 0; i < len; i++)
        {
            int index_index_1 = randint(0, len-1, data.rng);
            int tmp = indice[index_index_1];
            indice[index_index_1] = indice[len-1];
            indice[len-1] = tmp;
            int index_index_2 = randint(0, len-2, data.rng);
            get<0>(p_indice[i]) = indice[len-1];
            get<1>(p_indice[i]) = indice[index_index_2];
        }
    }
    else
    {
        /* other selection mechanims */
    }
}

void update_candidate_routes(Route &r, std::unordered_set<int> &inserted, Solution &s, vector<int> &candidate_r, Data &data)
{
    for (auto &node : r.node_list) {
        if (data.node[node].type == 1) inserted.insert(node);  //count customers in s
    }
    int i = 0;
    int len = int(candidate_r.size());
    while (i < len)
    {
        Route &r = s.get(candidate_r[i]);
        bool flag = true;
        for (auto &node : r.node_list)
        {
            if (node == data.DC) continue;
            if (inserted.count(node) == 1)
            {
                flag = false;
                break;
            }
        }
        if (!flag)
        {
            candidate_r.erase(candidate_r.begin() + i);
            len--;
        }
        else
            i++;
    }
}

void crossover(Solution &s1, Solution &s2, Solution &ch, double &child_fit, Data &data)
{

    if (data.no_crossover)
    {
        ch = s1;
    }

    vector<int> candidate_r_1(s1.len());
    std::iota(candidate_r_1.begin(), candidate_r_1.end(), 0);
    vector<int> candidate_r_2(s2.len());
    std::iota(candidate_r_2.begin(), candidate_r_2.end(), 0);
    int count = 0;
    std::unordered_set<int> inserted;
    inserted.reserve(data.customer_num + 1);

    while (true)
    {
        if (int(candidate_r_1.size()) == 0) break;
        int selected = randint(0, int(candidate_r_1.size())-1, data.rng);
        Route &r_1 = s1.get(candidate_r_1[selected]);
        ch.append(r_1);
        update_candidate_routes(r_1, inserted, s2, candidate_r_2, data);
        if (int(candidate_r_2.size()) == 0) break;
        selected = randint(0, int(candidate_r_2.size())-1, data.rng);
        Route &r_2 = s2.get(candidate_r_2[selected]);
        ch.append(r_2);
        update_candidate_routes(r_2, inserted, s1, candidate_r_1, data);
    }
    // call insertion
    if (data.cross_repair == RCRS)
    {
        data.n_insert = RCRS;
        data.ksize = data.k_crossover;
        // using random lambda and gamma
        data.lambda_gamma = std::make_tuple(rand(0, 1, data.rng), rand(0, 1, data.rng));
        // printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
        new_route_insertion(ch, data);
    }
    else if (data.cross_repair == TD)
    {
        data.ksize = data.k_crossover;
        data.n_insert = TD;
        new_route_insertion(ch, data);
    }
    else if (data.cross_repair == REGRET)
        regret_insertion(ch, data);
    
    child_fit = ch.cost;
}

void crossover(vector<Solution> &pop, Data &data, vector<tuple<int,int>> &p_indice, vector<Solution> &child, vector<double> &child_fit)
{
    cout << "Do crossover." << endl;
    int count = 0;
    for (auto &index_t : p_indice)
    {
        if (randint(0, 1, data.rng) == 0)
            crossover(pop[get<0>(index_t)], pop[get<1>(index_t)], child[count], child_fit[count] ,data);
        else
            crossover(pop[get<1>(index_t)], pop[get<0>(index_t)], child[count], child_fit[count], data);
        count++;
        //cout << "Child " << count << ". Parent Indice: (" << p1 <<\
                "," << p2 << "). Cost: " << child[count-1].cost << endl;
    }
}

void output(vector<Solution> &pop, vector<double> pop_fit, vector<int> pop_argrank,\
            Data &data, bool output_complete=false)
{
    int len = int(pop.size());
    double best_cost = pop_fit[pop_argrank[0]];
    double worst_cost = pop_fit[pop_argrank[len-1]];
    double avg_cost = mean(pop_fit, 0, len);
    printf("Avg %.2f, Best %.2f / m %d, Worst %.2f\n", avg_cost, best_cost, pop[pop_argrank[0]].len(), worst_cost);
    if (output_complete) {pop[pop_argrank[0]].output(data);}
    // for (int i = 0; i< len; i++){
    //     if (!pop[i].check(data)) {
    //         pop[i].output(data);
    //         exit(0);
    //     };
    // }
}

void local_search(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data)
{
    //printf("Do local search\n");

    int len = int(pop.size());
    for (int i = 0; i < len; i++)
    {
        if (rand(0, 1, data.rng) < data.ls_prob)
        {
                // cout << "Individual " << i+1 << ". Before Cost " << pop[i].cost << ".\n";

                Solution s = pop[i];
                // clock_t stime1 = clock();
                do_local_search(s, data);   
                // double used_sec1 = (clock() - stime1) / (CLOCKS_PER_SEC*1.0);
                // printf("%d, %.2lf, %.2lf, %.2lf sec\n", i, pop_fit[i], s.cost, used_sec1);
                if (s.cost - pop_fit[i] < -PRECISION) {
                    
                    pop_fit[i] = s.cost;
                    pop[i]=s;

                }
            
        }
    }
    argsort(pop_fit, pop_argrank, len);
}

void large_neighbourhood_search(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data)
{
    int len = int(pop.size());
    for (int i = 0; i < len; i++)
    {
        if (rand(0, 1, data.rng) < data.ls_prob)
        {
                // cout << "Individual " << i+1 << ". Before Cost " << pop[i].cost << ".\n";

                Solution s = pop[i];
                // clock_t stime1 = clock();

                // not using large neighborhood
                if (data.escape_local_optima == 0) return;
                
                // large neighborhood search

                int escape_local_optima = data.escape_local_optima;
                
                data.escape_local_optima = 0;

                static int tmp_solution_num = int(data.destroy_opts.size()) * int(data.repair_opts.size());
                if (data.rd_removal_insertion)
                    tmp_solution_num = 1;
                static std::vector<Solution> s_vector(tmp_solution_num);
                
                int no_improve = 0;
                while (no_improve < escape_local_optima)
                {
                    for (int i = 0; i < tmp_solution_num; i++)
                        s_vector[i] = s;
                    perturb(s_vector, data);
                    int best_index = -1;
                    double best_cost = double(INFINITY);
                    for (int i = 0; i < tmp_solution_num; i++)
                    {
                        Solution s_t = s_vector[i];            
                        do_local_search(s_t, data);   

                        //printf("%.2lf, %.2lf\n", s.cost, s_t.cost);

                        if (s_t.cost - s_vector[i].cost < -PRECISION) {
                            s_vector[i]=s_t;
                        }            

                        if (s_vector[i].cost - best_cost < -PRECISION)
                        {
                            best_index = i;
                            best_cost = s_vector[i].cost;
                        }
                    }
                    if (s_vector[best_index].cost - s.cost < -PRECISION)
                    {
                        s = s_vector[best_index];
                        no_improve = 0;
                    }
                    else
                    {
                        no_improve++;
                    }
                }
                data.escape_local_optima = escape_local_optima; 

                // double used_sec1 = (clock() - stime1) / (CLOCKS_PER_SEC*1.0);
                // printf("%d, %.2lf, %.2lf, %.2lf sec\n", i, pop_fit[i], s.cost, used_sec1);
                if (s.cost - pop_fit[i] < -PRECISION) {
                    
                    pop_fit[i] = s.cost;
                    pop[i]=s;

                }
            
        }
    }
    argsort(pop_fit, pop_argrank, len);
}

void replacement(vector<Solution> &pop, vector<tuple<int, int>> &p_indice, vector<Solution> &child, vector<double> &pop_fit, vector<int> &pop_argrank, vector<double> &child_fit, vector<int> &child_argrank, Data &data)
{
    int len = int(child.size());
    if (data.replacement == ONE_ON_ONE)
    {
        for (int i = 0; i < len; i++)
        {
            auto &indice_t = p_indice[i];
            auto p_1_indice = std::get<0>(indice_t);
            if (child[i].cost - pop[p_1_indice].cost < -PRECISION)
            {
                pop[p_1_indice] = child[i];
                pop_fit[p_1_indice] = pop[p_1_indice].cost;
            }
        }
    }
    else if (data.replacement == ELITISM_1)
    {
        pop[len-1] = pop[pop_argrank[0]];
        pop_fit[len-1] = pop_fit[pop_argrank[0]];
        for (int i = 0; i < len-1; i++)
        {
            pop[i] = child[child_argrank[i]];
            pop_fit[i] = pop[i].cost;
        }
    }
    else
    {
        /* more replacement */
    }
    for(int i = 0; i < len; i++)
    {
        child[i].clear(data);
    }
}

// Hybrid Memetic Search
void search_framework(Data &data, Solution &best_s, int level, clock_t stime0, double update_value)
{
    double cost_all_run = 0.0; 
    double time_all_run = 0.0;
    vector<double> solutions;
    vector<double> times;

    vector<Solution> individual(1);
    vector<Solution> pop(data.p_size);
    vector<Solution> child(data.p_size);
    
    individual[0].reserve(data);
    for (int i = 0; i < data.p_size; i++)
    {
        pop[i].reserve(data);  //reserve the max_num route
        child[i].reserve(data);
    }
    // fitness    
    
    vector<double> individual_fit(1);
    vector<double> pop_fit(data.p_size);
    vector<double> child_fit(data.p_size);
    // argsort result of fitness array
    
    vector<int> individual_argrank(1); 
    vector<int> pop_argrank(data.p_size); 
    vector<int> child_argrank(data.p_size); 

    // parent index in pop
    vector<tuple<int, int>> p_indice(data.p_size);

    // subproblem init
    if (level == 0) pop[0] = best_s;

    /* main body */
    bool time_exhausted = false;
    int run = 1;
    for (; run <= data.runs; run++)
    { 
        clock_t stime = clock();
        clock_t used = 0;
        double used_sec = 0.0;
        int no_improve = 0;
        int gen = 0;

        if (level == 1) {   //individual-based search

                printf("---------------------------------Run %d---------------------------\n", run);
                // construct a init solution
                initialization(individual, individual_fit, individual_argrank, data);
                used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                printf("already consumed %.2lf sec\n", used_sec); 
                
                // used = (clock() - stime) / CLOCKS_PER_SEC;
                // update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);

                local_search(individual, individual_fit, individual_argrank, data);
                
                used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                printf("already consumed %.2lf sec\n", used_sec);  
                // used = (clock() - stime) / CLOCKS_PER_SEC;
                // update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);
                printf("After local search\n");
                output(individual, individual_fit, individual_argrank, data);
                
                double cost_in_this_run = individual_fit[0];
                while (!termination(no_improve, data))
                {
                    gen++;
                    no_improve++;

                    if ((data.population_search && !data.individual_search) || (data.population_search && data.individual_search && termination(no_improve, data))){

                            // Decomposition Strategy
                            decomposition_cluster(individual, individual_fit, individual_argrank, data, stime);                           
                            used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                            printf("Decomposition done, already consumed %.2lf sec\n", used_sec);

                            // used = (clock() - stime) / CLOCKS_PER_SEC;
                            // update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);

                            printf("After decomposition\n");
                            output(individual, individual_fit, individual_argrank, data);

                            //-----------------------------------------------------------------------------------------------
                            used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                            printf("already consumed %.2lf sec\n", used_sec);  

                            used = (clock() - stime) / CLOCKS_PER_SEC;
                            update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);
                
                            if (data.tmax != NO_LIMIT && used_sec > clock_t(data.tmax))
                            {
                                time_exhausted = true;
                                break;
                            }
                            //-----------------------------------------------------------------------------------------------

                            local_search(individual, individual_fit, individual_argrank, data);
                            used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                            printf("already consumed %.2lf sec\n", used_sec);  
                            
                            printf("After local search\n");
                            output(individual, individual_fit, individual_argrank, data);

                            //-----------------------------------------------------------------------------------------------
                            used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                            printf("already consumed %.2lf sec\n", used_sec);  

                            used = (clock() - stime) / CLOCKS_PER_SEC;
                            update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);
                
                            if (data.tmax != NO_LIMIT && used_sec > clock_t(data.tmax))
                            {
                                time_exhausted = true;
                                break;
                            }
                            if (data.population_search && !data.individual_search) break; 
                            //-----------------------------------------------------------------------------------------------
                    }

                    
                    large_neighbourhood_search(individual, individual_fit, individual_argrank, data);

                    used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                    printf("already consumed %.2lf sec\n", used_sec);  

                    used = (clock() - stime) / CLOCKS_PER_SEC;
                    update_best_solution(individual[0], best_s, used, run, gen, data, level, stime0, update_value);

                    printf("After large neighbourhood search\n");
                    output(individual, individual_fit, individual_argrank, data);

                    if (individual_fit[0] - cost_in_this_run < -PRECISION)
                    {
                            no_improve = 0;
                            cost_in_this_run = individual_fit[0];
                    }

                    used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                    used = (clock() - stime) / CLOCKS_PER_SEC;

                    if (gen % OUTPUT_PER_GENS == 0)
                    {
                        printf("Gen %d done, no improvement for %d gens, already consumed %.2lf sec\n", gen, no_improve, used_sec);
                    }
                    // printf("-----------------------------------------------------------------\n");

                    if (data.tmax != NO_LIMIT && used_sec > clock_t(data.tmax))
                    {
                        time_exhausted = true;
                        break;
                    }
                }              

                printf("Run %d finishes\n", run);
                cost_all_run += individual_fit[0];
                time_all_run += used_sec;

                solutions.push_back(individual_fit[0]);
                times.push_back(used_sec);

                output(individual, individual_fit, individual_argrank, data);
                data.rng.seed(data.seed + run);
                // if (time_exhausted) {run++; break;}   

        }
        else{   // population-based search
                
                stime = stime0;

                initialization(pop, pop_fit, pop_argrank, data);
                used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                printf("already consumed %.2lf sec\n", used_sec); 
                // used = (clock() - stime) / CLOCKS_PER_SEC;
                // update_best_solution(pop[pop_argrank[0]], best_s, used, run, gen, data, level, stime0, update_value);

                local_search(pop, pop_fit, pop_argrank, data);
                
                used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                printf("already consumed %.2lf sec\n", used_sec);    
                // used = (clock() - stime) / CLOCKS_PER_SEC;     
                // update_best_solution(pop[pop_argrank[0]], best_s, used, run, gen, data, level, stime0, update_value);

                printf("After local search\n");
                output(pop, pop_fit, pop_argrank, data);

                double cost_in_this_run = pop_fit[pop_argrank[0]];
                while (!termination(no_improve, data))
                {
                    gen++;
                    // printf("---------------------------------Gen %d---------------------------\n", gen);
                    no_improve++;
                    // select parents
                    select_parents(pop, pop_fit, p_indice, data);
                    // crossover
                    crossover(pop, data, p_indice, child, child_fit);

                    used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                    printf("already consumed %.2lf sec\n", used_sec);

                    // do local search for children
                    local_search(child, child_fit, child_argrank, data);

                    // replacement
                    replacement(pop, p_indice, child, pop_fit, pop_argrank, child_fit, child_argrank, data);
                    // update best
                    argsort(pop_fit, pop_argrank, data.p_size);

                    used = (clock() - stime) / CLOCKS_PER_SEC;
                    update_best_solution(pop[pop_argrank[0]], best_s, used, run, gen, data, level, stime0, update_value);
                    if (pop_fit[pop_argrank[0]] - cost_in_this_run < -PRECISION)
                    {
                        no_improve = 0;
                        cost_in_this_run = pop_fit[pop_argrank[0]];
                    }

                    used_sec = (clock() - stime) / (CLOCKS_PER_SEC*1.0);
                    used = (clock() - stime) / CLOCKS_PER_SEC;

                    if (gen % OUTPUT_PER_GENS == 0)
                    {
                        printf("Gen: %d. ", gen);
                        output(pop, pop_fit, pop_argrank, data);
                        printf("Gen %d done, no improvement for %d gens, already consumed %.2lf sec\n", gen, no_improve, used_sec);
                    }

                    if (data.tmax != NO_LIMIT && used_sec > clock_t(data.tmax))
                    {
                        time_exhausted = true;
                        break;
                    }
                }

                printf("Run %d finishes\n", run);
                cost_all_run += pop_fit[pop_argrank[0]];
                time_all_run += used_sec;
                output(pop, pop_fit, pop_argrank, data);
                data.rng.seed(data.seed + run);
                // if (time_exhausted) {run++; break;}
 
        
            }
    }

    // output best solution
    if (level != 0) {

        printf("------------Summary-----------\n");
        if (BENCHMARKING_O_1_EVAL)
        {
            printf("Number of move eval calls: %d, average time: %d nanosecs\n", call_count_move_eval, mean_duration_move_eval);
        }
        best_s.output(data);
        if (!best_s.check(data)) exit(0);  // check if feasible, then save best solution and run time in file
        printf("Total %d runs, total consumed %.2lf sec\n", run-1, time_all_run);
        
        std::string timelimit = std::to_string(data.tmax);
        std::string subproblem_range = std::to_string(data.subproblem_range);
        //"./results/solution/jd_2/"
        std::string filename = data.output + data.problem_name + "_timelimit=" + timelimit + "_subproblem=" +  subproblem_range + ".txt";
        FILE *file_solution = fopen(filename.c_str(), "a");
        if (file_solution == nullptr) {
            perror("Failed to open file");
        }

        std::string output_s = "Details of the solution:\n";
        int len = best_s.len();
        for (int i = 0; i < len; i++)
        {
            Route &r=best_s.get(i);
            int flag = 0;
            double new_cost = 0.0;
            int index_negtive_first = -1;
            std::vector<int> &nl = r.node_list;
            output_s += "route " + std::to_string(i) +
                        ", node_num " + std::to_string(nl.size()) +
                        ", cost " + std::to_string(r.transcost) +
                        ", nodes:";
            update_route_status(nl, r.status_list, data, flag, new_cost, index_negtive_first); 
            int pre = -1;            
            for (int j = 0; j < nl.size(); j++)
            {  
                int node = nl[j];
                if (pre != -1){
                    for (int sub_node: data.hyperarc[pre][node]){
                          output_s += ' ' + std::to_string(sub_node);
                    }
                }
                output_s += ' ' + std::to_string(node);
                if (data.node[node].type != 1){
                    std::ostringstream stream1;
                    std::ostringstream stream2;
                    stream1 << std::fixed << std::setprecision(2) << r.status_list[j].arr_RD;
                    stream2 << std::fixed << std::setprecision(2) << r.status_list[j].dep_RD;
                    std::string Str1 = stream1.str();
                    std::string Str2 = stream2.str();
                    output_s += "(" + Str1 + ", "+ Str2 + ")";

                }
                pre = node;
            }
            output_s += '\n';
        }
        output_s += "vehicle (route) number: " + std::to_string(len) + '\n';

        std::ostringstream stream;
        
        stream << std::fixed << std::setprecision(2) << best_s.cost;
        std::string costStr = stream.str();

        output_s += "Total cost: " + costStr + '\n';
        const char* c_output_s = output_s.c_str();
        fprintf(file_solution, "%s", c_output_s);
        for (int i = 0; i < run - 1; i++){
            fprintf(file_solution, "%.2lf, %.2lf\n", solutions[i], times[i]);
        }
        fclose(file_solution);
       
        std::string filename_output = data.output +"output_1.txt";
        FILE *file = fopen(filename_output.c_str(), "a");
        if (file == nullptr) {
            perror("Failed to open file");
        }
        const char* c_str = data.problem_name.c_str();
        fprintf(file, "%s: %d, %.2lf, %.2lf, %.2lf, subproblem = %d, timelimit = %d\n", c_str, best_s.len(), best_s.cost, cost_all_run / (run-1), time_all_run / (run-1), data.subproblem_range, data.tmax);
        fclose(file);
    }

}