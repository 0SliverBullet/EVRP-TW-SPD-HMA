/******************************************************* 
Genetic Algorithm for Parallel Station Insertion
********************************************************/

#include "evolution.h"
#include "util.h"
#include <vector>
#include <algorithm>

int MAXITERA = 5;
int POPSIZE;
double DELTA;

struct Candidate
{
    int x[100];
    std::vector<int> node_list;
    double fitx;
} individual[301], best_r;

bool check_adjustment(int dimension, Route &r, Data &data, int idx) {
    bool evolution = true;
    // generate valid solutions
    individual[idx].node_list = r.customer_list;
    for (int j = dimension; j >= 1; --j) {
        if (individual[idx].x[j - 1] == 1) {
            int pre = individual[idx].node_list[j - 1];
            int post = individual[idx].node_list[j];
            individual[idx].node_list.insert(individual[idx].node_list.begin() + j, data.optimal_staion[pre][post][0]);
        }
    }

    int flag = 0;
    double new_cost = 0.0;
    int index_negtive_first = -1;
    update_route_status(evolution, individual[idx].node_list, r.status_list, data, flag, new_cost, index_negtive_first);
    
    if (flag != 1) return false;
    individual[idx].fitx = new_cost;
    return true;
}

bool initialization(int &dimension, Route &r, Data &data) {
    int cnt = 0, last = -1;

    for (int i = 0; i < POPSIZE; ++i) {


        if (last != i) {
            last = i;
            cnt = 0;
        } else {
            ++cnt;
            if (cnt > DELTA) {
                if (i == 0) {
                    return false;
                } else {
                    for (int k = i; k < POPSIZE; ++k) {
                        individual[k] = individual[randint(0, i - 1, data.rng)];
                    }
                    break;
                }
            }
        }
        
        std::generate(individual[i].x, individual[i].x + dimension, [&data]() {
            return rand(0, 1, data.rng) < 0.5 ? 1 : 0;
        });

        if (!check_adjustment(dimension, r, data, i)) {
            --i;
            continue;
        }
    }

    best_r = *std::min_element(individual, individual + POPSIZE, [](const Candidate& a, const Candidate& b) {
        return a.fitx < b.fitx;
    });

    return true;
}

bool evolution(int &dimension, Route &r, Data &data) {
    int flag = 0;
    double new_cost = 0.0;
    int index_negtive_first = -1;
    bool evolution = true;
    int cnt = 0, last = -1;

    for (int i = 0; i < POPSIZE; ++i) {

        
        if (last != i) {
            last = i;
            cnt = 0;
        } else {
            ++cnt;
            if (cnt > DELTA) {
                return i > 0;
            }
        }

        // selection
        int p1 = randint(0, POPSIZE - 1, data.rng);
        int p2 = randint(0, POPSIZE - 1, data.rng);
        
        for (int j = 0; j < dimension; ++j) {
            individual[POPSIZE].x[j] = individual[p1].x[j] ^ individual[p2].x[j];
            // crossover 
            if (rand(0, 1, data.rng) < 0.02) {
                individual[POPSIZE].x[j] ^= 1;
            }
            if (individual[POPSIZE].x[j] == 1 && rand(0, 1, data.rng) < 0.2) {
                individual[POPSIZE].x[j] = 0;
            }
            // mutation
        }

        if (!check_adjustment(dimension, r, data, POPSIZE)) {
            --i;
            continue;
        }
        
        // replacement
        if (individual[POPSIZE].fitx < individual[i].fitx) {
            individual[i] = individual[POPSIZE];
            if (individual[i].fitx < best_r.fitx) {
                best_r = individual[i];
            }
        }
    }
    return true;
}

bool parallel_station_insertion(int &dimension, Route &r, Data &data, double &evolution_cost) {
    
    POPSIZE = dimension * 3;
    DELTA = POPSIZE * MAXITERA;

    if (!initialization(dimension, r, data)) return false;

    int gen = 0;
    while (gen < MAXITERA) {
        if (!evolution(dimension, r, data)) break;
        ++gen;
    }

    r.node_list = best_r.node_list;
    evolution_cost = best_r.fitx;
    return true;
}