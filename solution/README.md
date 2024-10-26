# solutions obtained in the experiments (10 independent runs)

The solution filename format is "`[InstanceFilename]_timelimit=[TIME]_subproblem=[K_SUBPROBLEM].txt`," where

- `InstanceFilename`: the same as in the filenames in `akb_instances` and `jd_instances`;
- `TIME`: the maximum computation time limit in one run; 
- `K_SUBPROBLEM`: `subproblem=1` means no decomposition; otherwise, it indicates decomposition.

## Solution File Structure

The solution file consists of two parts:

1. **The best solution** found among 10 independent runs, along with its details;
2. 10 solutions: the objective value $TC$ and computation time $time$ **for each run**.

For example, in `c101C5_timelimit=105_subproblem=1.txt`, **the first part** is:

```
Details of the solution:
route 0, node_num 7, cost 151.486134, nodes: 0(77.75, 77.75) 8(53.73, 77.75) 5 1 6(9.75, 69.15) 4 0(9.68, 9.68)
route 1, node_num 5, cost 106.261318, nodes: 0(77.75, 77.75) 2 7(33.59, 77.75) 3 0(15.65, 15.65)
vehicle (route) number: 2
Total cost: 2257.75
```

where `route i` indicates the i-th route $\boldsymbol{R}_i$ in solution $\boldsymbol{S}$;

`node_num` is the number of nodes (depot, customers, and charging stations) visited in $\boldsymbol{R}_i$, i.e., $L_i$;

`cost` is the transportation cost of $\boldsymbol{R}_i$, i.e, $\mu_2 \cdot TD(\boldsymbol{R}_i)$;

`nodes:` $n_1, n_2, n_2, ... n_{L_i}$  is  $\boldsymbol{R}_i = (n_1, n_2, \ldots , n_{L_i})$. Note that there are some $(y_{n_j}, Y_{n_j})$ following $n_j$. Recall that in our paper, the battery energy state of the EV upon arrival at and departure from $n_j$ is denoted as $y_{n_j}$ and $Y_{n_j}$ in the partial recharges scenarios. We explicitly show $(y_{n_j}, Y_{n_j})$ when visiting **the depot** and **the stations that the EV visits to recharge** in the route. In other words, for **the customers** and **the stations that the EV visits but does not recharge**, we omit showing $(y_{n_j}, Y_{n_j})$ since $y_{n_j} = Y_{n_j}$ is trivial;

`vehicle (route) number` is the number of vehicles used, i.e. $K$;

`Total cost` is the objective value $TC$, i.e., $TC(\boldsymbol{S})=\mu_1 \cdot K+\mu_2 \cdot \sum_{i=1}^{K}TD(\boldsymbol{R}_i)$.

**The second part** is:

```
2257.75, 0.39
2257.75, 0.37
2257.75, 0.37
2257.75, 0.37
2257.75, 0.38
2257.75, 0.38
2257.75, 0.38
2257.75, 0.39
2257.75, 0.37
2257.75, 0.37
```

where the left columns contain the objective value $TC$ of 10 solutions in each run, while the right columns record the computation time $time$ to obtain a solution in each run.
