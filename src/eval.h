#pragma once
#include "solution.h"
#include "data.h"
#include "move.h"
#include "evolution.h"
#include <chrono>
void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, bool &flag, double &cost);

void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, int &flag, double &cost);

void chk_route_O_n(Route &r, Data &data, bool &flag, double &cost);

void update_route_status(std::vector<int> &nl, std::vector<status> &sl, Data &data, int &flag, double &cost, int &index_negtive_first);

void update_route_status(bool &evolution, std::vector<int> &nl, std::vector<status> &sl, Data &data, int &flag, double &cost, int &index_negtive_first);

bool eval_move(Solution &s, Move &m, Data &data, double &base_cost);

static inline bool check_capacity(const Attr &a, const Attr &b, Data &data)
{
    return std::max(a.C_H + b.C_E, a.C_L + b.C_H) - data.vehicle.capacity <= 0;
}

static inline bool check_tw(const Attr &a, const Attr &b, Data &data)
{
    return (a.T_E + a.T_D + data.time[a.e][b.s] - b.T_L) <= 0;
}

std::vector<int> apply_move(Solution &s, Move &m, Data data);
bool cal_score_station(bool type, std::vector<int> &feasible_pos, std::vector<int> &station_pos, std::vector<double> &score, Route &r,Data &data,int index_last_f0, int index_negtive_first);
double criterion_station(Route &r, Data &data, int node, int pos);
bool parallel_sequential_station_insertion(Solution &item, Route &r, Data &data, int &j);
bool sequential_station_insertion(int &flag, int &index_negtive_first, Route &r, Data &data, std::vector<std::pair<int,int>> &station_insert_pos, double &heuristic_cost);
void sequential_station_improvement(double &cost, Data &data, Route &r);