#include "parameters/random_generators.h"
#include "parameters/optimization.h"
#include "parameters/hardcoded.h"
#include "parameters/others.h"

#include "sideFunctions/sorting.h"

#include <iostream>

// Initialize the LNS algorithm 
void initializeLNS (){
    ndestroy = rand_destroy_count(rng_engine);
    //std::cout << "Iteration:	" << it + 1 << " ---------------" << endl;

    // Initialize xsol and ysol
    for (int i = 0; i < nBuses; i++) {
        for (int j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) ysol[i][j][k] = 0;
            for (int p = 0; p < C; p++) xsol[i][p][j] = xk[i][p][j];
        }
        // calculate the used capacity and time window
        bCapi[i] = 0;

        earlyTW[i] = 1000000000;
        lateTW[i] = -1;
        for (int p = 0; p < C; p++) {
            for (int j = 0; j < Stations; j++) {
                if (xsol[i][p][j] == 1) {
                    bCapi[i]++;
                    if (arrivals[p] > lateTW[i])lateTW[i] = arrivals[p];
                    if (arrivals[p] < earlyTW[i])earlyTW[i] = arrivals[p];
                    break;
                }
            }
        }
    }
    //Destroyed is the array keeping track of the passangers that are removed, at each iteration the array is reset 
	for (int p = 0; p < C; p++) destroyed[p] = -1;

}

// Destroy part of the solution 
void destroyOperator(){
    for (int l = 0; l < ndestroy;) {
        di = rand_passenger(rng_engine); // passanger to remove is chosen randomly

        //only if the chosen destroy passenger hasnt been destroyed yet
        if (destroyed[di] == -1) {

            //reduced the used capacity of the bus i
            pbi = pst = -1;
            for (int i = 0; i < nBuses; i++) {
                for (int j = 0; j < Stations; j++) {
                    if (xsol[i][di][j] == 1) {
                        pst = j; //previous station 
                        pbi = i; // previous bus (of destroyed passenger)
                        break;
                    }
                }
                if (pbi == i) break;
            }
            if (pbi == -1) {
                std::cout << " oops passenger " << di << " not found \n ------------ INFEASIBLE for arrival times  ------------";
                exit(0);
            }

            //determine the bus to assign passenger again

            //rank buses according to arrival times of the passengers it is carrying
            for (int i = 0; i < nBuses; i++) {
                rank_buses[i] = i;
                value_buses[i] = 0;
                int n = 0;
                for (int p = 0; p < C; p++) {
                    sumx = 0;
                    for (int j = 0; j < Stations; j++) sumx += xk[i][p][j];
                    if (sumx == 1) {
                        n++;
                        value_buses[i] += arrivals[p];
                    }
                }
                if (n != 0)value_buses[i] = abs(value_buses[i] / n - arrivals[di]);
                else value_buses[i] = 0;
            }
            quickSort(rank_buses, value_buses, 0, nBuses - 1);

            it_bi = rand_bus_weighted(rng_engine);
            //cout << "need " << it_bi << endl;
            for (int i = nBuses; i >= 1; i--) {
                if (it_bi <= rp[i - 1] && it_bi > rp[i]) {
                    it_bi = nBuses - i;
                    //cout << "suceeded: " << it_bi << endl;
                    break;
                }
                //cout << "tried between: " << rp[i] << " and " << rp[i-1]<< endl;
            }

            bi = rank_buses[it_bi];
            //std::cout << " destrotyed passenger " << di << " try next bus " << bi<< endl;
            //for (int ii = 0; ii < nBuses; ii++)std::cout << bCapi[ii] << endl;
            //cout << bi << endl;
            if (bCapi[bi] >= bCapacity || (int)(arrivals[di] - earlyTW[bi]) > d_time1 + d_time2 || (int)(lateTW[bi] - arrivals[di]) > d_time1 + d_time2) {
                // make sure capacity Bus constraint is respected
                trybuses = 1;
                while (bCapi[bi] >= bCapacity || (int)(arrivals[di] - earlyTW[bi]) > d_time1 + d_time2 || (int)(lateTW[bi] - arrivals[di]) > d_time1 + d_time2) {
                    if (trybuses >= nBuses) {
                        bi = pbi;
                        //cout << "nope" << endl;
                        break;
                    }
                    it_bi = (it_bi + 1) % nBuses;
                    bi = rank_buses[it_bi];
                    //cout << " new try bus " << bi << endl;
                    trybuses++;
                }
            }

            //std::cout << " passenger " << di << " went to bus  " << bi << " from bus " << pbi << endl;
            //if (bi == pbi) continue; // if the same bus is chosen agin keep trying 
            //cout << "Passenger: " << di << endl;
            //cout << " old bus: " << pbi << endl;
            //cout << " new bus: " << bi << endl;
            destroyed[di] = bi;
            // update used capacity 
            bCapi[bi]++;
            bCapi[pbi]--;

            //cout << "passenger " << di << " at station " << pst << " removed from bus " << pbi << " and assigned to bus " << bi <<  endl;

            //------------ DESTROY -----------------------------------
            xsol[pbi][di][pst] = 0; //if passenger went to mandatory station
            l++;
        }

    }

}

// Rebuild part of the destroyed solution 
void rebuildOperator(){
    for (int i = 0; i < nBuses; i++) {

        //cout << " Bus " << i + 1 << endl;

        //ADJUST x  VARIABLES *************************************************
        for (int p = 0; p < C; p++) {
            //for the passengers that were taken out of the solution and are assigned to this bus i
            if (destroyed[p] == i) {
                //adjust in the x variable TO NEW STATION (account for walking constraint)--------------- 

                if (traveltimep[p][closestPS[p][0]] > d) {
                    std::cout << " Infeasible solution, for walking times " << std::endl;
                    infeas = true;
                    break;
                }
                if (closestPS[p][0] <= N - 1) {
                    newj = closestPS[p][0];
                    xsol[i][p][newj] = 1;
                }
                else {
                    which = 0;
                    inGraph = false;
                    for (int c = 0; c < C; c++) {
                        if (xsol[i][c][closestPS[p][which]]) {
                            inGraph = true;
                            break;
                        }
                    }
                    if (!inGraph) {
                        //howmuch = (1 - traveltimep[p][closestPS[p][0]] / traveltimep[p][closestPS[p][1]]) * 100;
                        //howmuch = (100-secondS);
                        //cout << howmuch << endl;
                        decide = rand_percentage(rng_engine);
                        if (decide > secondS && closestPS[p][1]<=d) which = 1;
                        else if (decide < secondS && decide > secondS-thirdS && closestPS[p][2] <= d) which = 2;
                    }

                    //which = 0;
                    newj = closestPS[p][which];
                    xsol[i][p][newj] = 1;

                }

                //cout << "REBUILD on bus " << i+1 <<" new : " << newj << endl;
            }
        }

        if (infeas) break;

        //cout << "BUS " << i << endl;

        /*
        //cout << "BUS " << i << endl;

        cout << " X var after " << endl;
        for (p = 0; p < C; p++) {
            for (j = 0; j < Stations; j++) {
                cout << xsol[i][p][j] << " ";
            }
            cout << endl;
        }
        cout << " Y var Before " << endl;
        for (p = 0; p < Stations; p++) {
            for (j = 0; j < Stations; j++) {
                cout << ysol[i][p][j] << " ";
            }
            cout << endl;
        }
        */

        //ADJUST y  VARIABLES  ************************************************
        //(ONLY IF NEW PASSENGER GOES TO OPTIONAL STATION)

        int j = 0;
        imand = 0;

        // not feasible, forgets stations !!!! ----------------------------------------------------------------------------------------------------
        while (j != N - 1) {
            //cout << j << endl;
            // determine the last mandatory station visited 
            if (j < N - 1) imand = j;

            //cout << j << " Has NO successor  "  ;
            firstmin = 100000000, firstminj = -1;
            for (int k = N + imand * M; k < N + (imand + 1) * M; k++) { // first check the optional stations 
                //check if node is in chosen subset
                inGraph = false;
                for (int p = 0; p < C; p++) {
                    if (xsol[i][p][k] > 0) {
                        inGraph = true;
                        break;
                    }
                }

                //check if node is already a successor
                inSuccessor = false;
                for (int l = 0; l < Stations; l++) {
                    if (ysol[i][l][k] == 1) {
                        inSuccessor = true;
                        //cout << "successor 1 is " << k + 1 << endl;
                        break;
                    }
                }

                //calculate closest eligible stop (it is among the chosen stops and it is not a succeesor yet )
                if (inGraph && j != k && !inSuccessor) {
                    //if ( it == 657841 && i == 0)cout << " k is " << k << " and distance is " << traveltimes[j][k] << " and is eligible " << endl;
                    if (firstmin > traveltimes[j][k]) {
                        firstmin = traveltimes[j][k];
                        firstminj = k;
                    }
                }
            }
            if (firstminj == -1) { // if there are no elegible optional stations
                //cout << " no optional in same cluster available"<< endl;
                for (int k = 1; k < N; k++) { //then check the mandatory ones

                    //check if node is already a successor
                    inSuccessor = false;
                    for (int l = 0; l < Stations; l++) {
                        if (ysol[i][l][k] == 1) {
                            inSuccessor = true;
                            //cout << "successor 2 is " << k + 1 << endl;
                            break;
                        }
                    }

                    //calculate closest eligible stop (it is among the chosen stops and it is not a succeesor yet )
                    if (j != k && !inSuccessor) {
                        //cout << " k is " << k << " and distance is " << traveltimes[j][k] << " and is eligible " << endl;
                        if (firstmin > traveltimes[j][k]) {
                            firstmin = traveltimes[j][k];
                            firstminj = k;
                            //cout << " k is " << k << endl;
                        }
                    }
                }

                //if (firstminj == -1) cout << " no mandatory available" << endl;
                //and check the next or previous cluster of optionals
                for (int k = std::max(N + (imand - 1) * M, N); k < std::min(N + (imand + 2) * M, Stations); k++) { // first check the optional stations 
                //check if node is in chosen subset
                    inGraph = false;
                    for (int p = 0; p < C; p++) {
                        if (xsol[i][p][k] == 1) {
                            inGraph = true;
                            break;
                        }
                    }

                    //check if node is already a successor
                    inSuccessor = false;
                    for (int l = 0; l < Stations; l++) {
                        if (ysol[i][l][k] == 1) {
                            inSuccessor = true;
                            //cout << "successor  3 is " << k + 1 << endl;
                            break;
                        }
                    }

                    //calculate closest eligible stop (it is among the chosen stops and it is not a succeesor yet )
                    if (inGraph && j != k && !inSuccessor) {
                        //cout << " k is " << k << " and distance is " << traveltimes[j][k] << " and is eligible " << endl;
                        if (firstmin > traveltimes[j][k]) {
                            firstmin = traveltimes[j][k];
                            firstminj = k;
                            //cout << " k is " << k << endl;
                        }
                    }
                }

            }
            ysol[i][j][firstminj] = 1;
            ysol[i][firstminj][j] = 0; //avoid loops
            j = firstminj;
            //cout << j << endl;
        }

        //Calculate earliest desired arrival time
        earliestArr = 1000000;
        latestArr = -1;
        for (int p = 0; p < C; p++) {
            sumx = 0;
            for (j = 0; j < Stations; j++) sumx += xsol[i][indexpt[p]][j];
            if (sumx == 1) {
                earliestArr = arrivals[indexpt[p]];
                break;
            }
        }
        for (int p = C - 1; p >= 0; p--) {
            sumx = 0;
            for (j = 0; j < Stations; j++) sumx += xsol[i][indexpt[p]][j];
            if (sumx == 1) {
                latestArr = arrivals[indexpt[p]];
                break;
            }
        }

        // fill in the A VARIABLE  ********************************************
        BusPa.clear();
        Asol[i] = 0;
        //Dsol[i] = 0;
        for (j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) {
                Asol[i] += (traveltimes[j][k] + delta) * ysol[i][j][k];
            }
            for (int c = 0; c < C; c++) {
                Asol[i] += (double)tau * xsol[i][c][j];
                if (xsol[i][c][j] == 1) {
                    BusPa.push_back(arrivals[c]);
                }
            }
        }

        if (BusPa.size() != 0) {
            Arr = findMedian(BusPa);
            if (Arr <= std::max(latestArr - d_time1, earliestArr)) {
                Arr = std::max(latestArr - d_time1, earliestArr);
                //cout << " too low \n";
            }
            else if (Arr >= std::min(earliestArr + d_time2, latestArr)) {
                Arr = std::min(earliestArr + d_time2, latestArr);
                //cout << " too high \n";
            }
        }
        else {
            Arr = Asol[i];
            //cout << " No passengers \n";
        }

        //std::cout << " latest + d1: " << latestArr - d_time1 << ", earliest + d2: " << earliestArr + d_time2 << endl;
        //Here still do something to account infeasiblity 
        Dsol[i] = Arr - Asol[i];
        Asol[i] = Arr;
        infeas = false;
        if (Dsol[i] < 0) {
            std::cout << " Infeasible solution, for arrival times " << std::endl;
            infeas = true;
            break;
        }

        // COSTS --------------------------------------------------------------
        for (j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) {
                nextcost += c1 * ysol[i][j][k] * (traveltimes[j][k] + delta);
            }

            for (int p = 0; p < C; p++) {
                nextcost += xsol[i][p][j] * (c1 * (double)tau + c2 * traveltimep[p][j]);
            }
        }

        for (int p = 0; p < C; p++) {
            sumx = 0;
            for (j = 0; j < Stations; j++) sumx += xsol[i][p][j];
            if (sumx > 0) {
                nextcost += c3 * fabs(arrivals[p] - Asol[i]);
            }
        }
    }
}

// Acceptance criteria
void acceptanceCriteria(std::clock_t start_time){
    if ((int)(nextcost*100) < (int)(cost*100)) {
        bestit = it;
        cost = nextcost;
        last= (double)(clock() - start_time) / CLOCKS_PER_SEC;
        rt.push_back(last);
        prog.push_back(cost);
        
        // assign new solutions
        for (int i = 0; i < nBuses; i++) {
            cap = 0;
            Ak[i] = Asol[i];
            Dk[i] = Dsol[i];
            for (int j = 0; j < Stations; j++) {
                for (int k = 0; k < Stations; k++) yk[i][j][k] = ysol[i][j][k];

                for (int p = 0; p < C; p++) {
                    xk[i][p][j] = xsol[i][p][j];
                    //if (xk[i][p][j] == 1) cap++;
                }
            }
            //std::cout << " Bus " << i + 1 << " cap: " << cap << " max : " << bCapacity << endl;
        }
    }
}

// 2opt improvement after a destroy-repair cycle
void improvementLNS(){
    //SA PARAMETERS -----------------------------------------------------------------------------
    //number of nearest neighbors 
    maxdist = -1;
    for (int i = 0; i < m; i++) {
        if (traveltimes[closestS[i][0]][i + 1] > maxdist)maxdist = traveltimes[closestS[i][0]][i + 1];
    }
    T = maxdist;
    alpha = 0.95;
    T_end = 0.01;
    L = 100;


    //INITIATE PARAMETERS
    std::uniform_real_distribution<double> r(0, 1);
    m = std::min(M - 1 + 2 * M + 2, Stations);
    std::uniform_int_distribution<int> NEXT(0, m);

    for (int i = 0; i < nBuses; i++) {

        //FILL IN ROUTE
        int j = 0;
        route.push_back(j);
        int n = 1;
        //std::cout << route[n - 1] << " ";
        while (j != N - 1) {
            for (int k = 1; k < Stations; k++) {
                if (yk[i][j][k] == 1) {
                    yk[i][j][k] = 0;
                    route.push_back(k);
                    n++;
                    j = k;
                    //std::cout << k << " ";
                    break;
                }
            }
        }
        //std::cout << "\n----- n: " << n << endl;

        //number of opt operations
        for (int p = 0; p < 5000; p++) {
            j = rand_station(rng_engine);
            index1 = giveIndex(route, j, n);
            while (j == N - 1 || index1 == n) {
                j = rand_station(rng_engine);
                index1 = giveIndex(route, j, n);
            }

            int l = closestS[j][NEXT(rng_engine)];
            index2 = giveIndex(route, l, n);
            while (l == j || l == N - 1 || index2 == n) {
                l = closestS[j][NEXT(rng_engine)];
                index2 = giveIndex(route, l, n);
            }

            if (index1 > index2) {
                start = index2;
                end_i = index1;
            }
            else {
                start = index1;
                end_i = index2;
            }
            nextindex = end_i + 1;

            currentcost = traveltimes[route[start]][route[start + 1]] + traveltimes[route[end_i]][route[nextindex]];
            ncost = traveltimes[route[start]][route[end_i]] + traveltimes[route[start + 1]][route[nextindex]];

            //std::cout << " Current cost: " << currentcost << " New Cost: " << newcost << " \n";

            //--------------------------------------------------------------------SA --------------------------------------------------------------------------
            dE = (ncost - currentcost);

            if (dE < 0) {
                //std::cout << " Found better dE =" << dE << " on bus " << i << " +++++++++++++++++++++++\n";
                //std::cout << " it : " << p << endl;
                cost += c1 * dE;
                Dk[i] -= dE;
                mid = (end_i - start) / 2;

                //2opt
                for (cc = 1; cc <= mid; cc++) {
                    temp = route[start + cc];
                    route[start + cc] = route[end_i + 1 - cc];
                    route[end_i + 1 - cc] = temp;
                }
            }
        }

        for (j = 0; j < n - 1; j++) {
            yk[i][route[j]][route[j + 1]] = 1;
        }

        route.clear();
    }
		
}
