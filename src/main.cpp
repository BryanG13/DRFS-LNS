#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <stdlib.h>
#include <time.h>


// include the side funcion
#include "sideFunctions/reporting.h"
#include "sideFunctions/sorting.h"


// parameters
#include "parameters/hardcoded.h"
#include "parameters/optimization.h"
#include "parameters/others.h"
#include "parameters/random_generators.h"


// Optimzation
#include "optimization/LNS.h"
#include "optimization/initialSolution.h"


using namespace std;

int main() {

    //************************************************* PRE PROCESSING ********************************************************
    // small pre processing of parameters
    processingParameters();
    saveInstance();

    // Initialize random distributions after parameters are loaded
    initializeRandomDistributions();

    // initialize a solution raw pointer
    initilalizeX();

    // std::cout << "cost: " << cost << endl;

    ofstream txt_runs("Data/results/Runs_" + to_string(instance) + ".txt");

    for (int run = 0; run < nRUN; run++) {
        //************************************************* INITIALIZE ********************************************************
        // time the heuristic
        clock_t start_time;
        start_time = clock();

        // initialize cost
        cost = 0;

        // Get initial solution
        initialSolution();
        twoOptImprovement(start_time);

        // std::cout << " Initial cost: " << cost << endl;

        //********************************************************* ||||||||||| HEURISTIC ||||||||||||||| **********************************************************

        // intialize for LNS
        it = 0;
        nextcost = 0;
        infeas = false;

        while (it - bestit <= f_IT) {
            initializeLNS();

            //**************************************** || DESTROY (and assign new bus for passengers) || *************************
            destroyOperator();

            // Set nexcost back to 0
            nextcost = 0;

            // std::cout << "destroyed \n";
            //***************************************** || REBUILD FUNCTION || ****************************************************
            rebuildOperator();

            // if one of the buses is infeasible, go to next proposal for a silution
            if (infeas) {
                it++;
                infeas = false;
                continue;
            }

            // ACCEPTANCE CRITERIA ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            acceptanceCriteria(start_time);

            it++;
        }
        // std::cout << "END ---------------------------------------------------------------- \n it: " << it << "best it: " << bestit<< endl;

        // simple 2-opt to improve routing
        improvementLNS();

        // For reporting
        logRun(start_time, run, txt_runs);
    }

    // some printing
    std::cout << endl
              << "+++++++++++++++++++++++++++++ \n Best solution for inst. " << instance << 
			  ":  \n cost: " << best_cost << 
			  "\n run-time: " << best_rt << 
			  "s \n last improvement: " << bestlast << 
			  "s \n+++++++++++++++++++++++++++++\n";
    txt_runs.close();

    // Write to file Ô
    writeSolution();

    // remove memory
    deletePointers();
}