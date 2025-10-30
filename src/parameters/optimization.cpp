
// parameters
#include "parameters/hardcoded.h"
#include <climits>
#include <vector>

// Decision variables: passenger-to-station assignment (x)
bool ***xsol;  // solution being evaluated
bool ***xk;    // current best solution
bool ***xbest; // overall best solution

// Decision variables: station-to-station routing (y)
bool yk[nBuses][Stations][Stations];    // current best routing
bool ysol[nBuses][Stations][Stations];  // solution being evaluated routing
bool ybest[nBuses][Stations][Stations]; // overall best routing

// Arrival times
double Ak[nBuses];    // current best arrival time for each bus
double Dk[nBuses];    // current best idle/delay time before departure for each bus
double Dsol[nBuses];  // solution idle/delay time
double Asol[nBuses];  // solution arrival time
double Abest[nBuses]; // overall best arrival time
double Dbest[nBuses]; // overall best idle/delay time

int bCapi[nBuses]; // used capacity of each bus in current solution

std::vector<double> rt;           // runtime at each improvement
std::vector<double> best_srt;     // best solution runtimes
std::vector<int> prog, best_prog; // cost progression over iterations

// Algorithm variables
double cost;                   // total objective function value
double currentcost, ncost, dE; // current cost, new cost, and cost difference for SA
double elapsed_time;           // total elapsed time for the run
int it = 0;                    // iteration counter for LNS
double nextcost = 0;           // cost of proposed solution

// Initialization phase variables
int pa = 0;                  // number of passengers assigned so far
int cap = 0, countcap = 0;   // current bus capacity usage and counter
int minDist = INT_MAX;       // minimum distance value
int mini = -1;               // index of minimum
int sumx = 0;                // sum of x values (for checking if passenger is assigned)
double sum = 0, sum_min = 0; // utility sums

int pindex[C];                      // station assignment for each passenger during initialization
double earliestArr, latestArr, Arr; // earliest, latest, and chosen arrival times
int minps = 10000000;               // minimum passenger-to-station distance
int minpsi = 0;                     // index of closest station for passenger

double timewindow; // time window between passengers
double firsttw;    // arrival time of first passenger on bus
int p_0, p_new;    // previous and new passenger indices for median calculation
int bestit = 0;    // iteration number of best solution found

int destroyed[C]; // tracks which bus each destroyed passenger is reassigned to (-1 if not destroyed)
int di;           // passenger index to destroy/remove
int bi;           // new bus for the removed passenger
int pbi;          // previous bus the removed passenger was on
int pst;          // previous station the removed passenger went to

double firstmin, secmin, thirdmin; // first, second, third minimum distance values
int firstminj, secminj, thirdminj; // indices of first, second, third minimum
int newj;                          // new station for reassigned passenger
bool infeas = false;               // flag for infeasible solution
bool inGraph, inSuccessor;         // flags for checking if station is in route

// Bus selection variables
double value_buses[nBuses];  // ranking values for buses (based on avg arrival time difference)
int rank_buses[nBuses];      // sorted bus indices by ranking value
int it_bi;                   // selected bus index from weighted distribution
double earlyTW[nBuses];      // earliest passenger arrival time on each bus
double lateTW[nBuses];       // latest passenger arrival time on each bus
int trybuses, decide, which; // counters and decisions for bus/station selection

// Simulated Annealing parameters and 2-opt
int maxdist = -1;    // maximum distance between nearest neighbors
double T;            // initial temperature
double alpha = 0.95; // cooling rate
double T_end = 0.01; // end temperature
int L = 100;         // iterations per temperature
int m;               // number of nearest neighbors to consider

// Best solution tracking
double best_cost = INT_MAX; // best cost found across all runs
double best_rt = INT_MAX;   // runtime of best solution
double last, bestlast;      // time of last improvement

void deletePointers() {
    for (int i = 0; i != nBuses; i++) {
        for (int j = 0; j != C; j++) {
            delete[] xk[i][j];
            delete[] xsol[i][j];
            delete[] xbest[i][j];
        }
        delete[] xk[i];
        delete[] xsol[i];
        delete[] xbest[i];
    }
    delete[] xk;
    delete[] xsol;
    delete[] xbest;
}

void initilalizeX() {
    xsol = new bool **[nBuses];  // solution being evaluated
    xk = new bool **[nBuses];    // current best solution
    xbest = new bool **[nBuses]; // overall best solution
    for (int i = 0; i != nBuses; i++) {
        xk[i] = new bool *[C];
        xsol[i] = new bool *[C];
        xbest[i] = new bool *[C];
        for (int j = 0; j != C; j++) {
            xk[i][j] = new bool[Stations];
            xsol[i][j] = new bool[Stations];
            xbest[i][j] = new bool[Stations];
        }
    }
}