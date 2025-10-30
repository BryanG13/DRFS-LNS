#include <iostream>

#include "parameters/optimization.h"
#include "parameters/others.h"
#include "parameters/random_generators.h"

#include "sideFunctions/sorting.h"

// intialize some parameters
inline void startInitalSolution() {
    // initialize indices
    cc = 0;
    for (int p = 0; p < C; p++) {
        pindex[p] = 0;
    }

    // intialize others
    pa = 0;
    cap = 0, countcap = 0;
    minDist = INT_MAX;
    mini = -1;
    sumx = 0;
    sum = 0;
    sum_min = 0;
    minps = 10000000;
    minpsi = 0;
}

// algorithm for initial solution
void initialSolution() {
    int n = 0;
    startInitalSolution();

    // for each bus avaialable
    for (int i = 0; i < nBuses; i++) {
        // cout << "Bus " << i + 1 << endl;
        cap = 0;
        BusPa.clear();

        // intialize vector y
        for (int j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) {
                yk[i][j][k] = 0;
            }
        }

        // intialize vector x
        for (int p = 0; p < C; p++) {
            for (int k = 0; k < Stations; k++) {
                xk[i][p][k] = 0;
            }
        }

        // fill in the x variable
        for (int p = pa; p < C; p++) {
            if (cap > 0) {
                timewindow = abs(firsttw - arrivals[indexpt[p]]);
            }
            else {
                firsttw = arrivals[indexpt[p]];
                timewindow = 0;
            }
            // std::cout << "passenger " << indexpt[p] << " time window is " << timewindow  << " <= " << (d_time1 + d_time2)  << endl;
            // cout << cap << "<" << bCapacity << endl;
            // cout << pa << "<" << C << endl;
            if (cap < bCapacity && pa < C && (int)timewindow <= d_time1 + d_time2) {
                minpsi = closestPS[indexpt[p]][0];
                if (traveltimep[indexpt[p]][minpsi] <= d) {
                    xk[i][indexpt[p]][minpsi] = 1;
                    BusPa.push_back(arrivals[indexpt[p]]);
                    // cout << "Chosen \n";
                    cap++;
                    pindex[pa] = minpsi;
                    pa++;
                    // cout << "passenger " << indexpt[p] << " arrival " << arrivals[indexpt[p]] << endl;
                }
                else {
                    std::cout << "INFEASIBLE for walking \n";
                    std::cout << "passenger " << indexpt[p] + 1 << " has a minimum walking time of " << traveltimep[indexpt[p]][minpsi] << std::endl;
                    exit(0);
                }
            }
        }

        // all mandatory Staions
        for (int k = 0; k < N - 1; k++) {
            yk[i][k][k + 1] = 1;
        }

        int p = 0;
        // fill in the y variable
        for (const auto &p : pindex) {
            if (p >= N) {
                minDist = INT_MAX;
                mini = -1;
                bool already = false;
                for (int j = 0; j < Stations; j++) {
                    if (yk[i][j][p] == 1) {
                        already = true;
                        break;
                    }
                }
                if (already) {
                    continue;
                }
                // cout << "Optional: " << p << endl;
                for (int j = 0; j < N; j++) {
                    if (traveltimes[j][p] < minDist && j != p) {
                        minDist = traveltimes[j][p];
                        mini = j;
                    }
                }
                if ((p - N) / M + 1 <= mini) {
                    mini--;
                }
                // cout <<"Mandatory " << mini << endl;

                if (yk[i][mini][mini + 1] != 0) {
                    yk[i][mini][mini + 1] = 0;
                    yk[i][mini][p] = 1;
                    yk[i][p][mini + 1] = 1;
                }
                else {
                    int gotcha = -1;
                    for (const auto &pp : pindex) {
                        if (pp >= N && yk[i][mini][pp] == 1 && pp != p) {
                            gotcha = pp;
                            break;
                        }
                    }
                    yk[i][mini][gotcha] = 0;
                    yk[i][mini][p] = 1;
                    yk[i][p][gotcha] = 1;
                }
            }
        }

        // fill in the A variable
        Ak[i] = 0;
        for (int j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) {
                Ak[i] += (traveltimes[j][k] + delta) * yk[i][j][k];
            }
            for (int c = 0; c < C; c++) {
                Ak[i] += (double)tau * xk[i][c][j];
            }
        }

        // Calculate earliest desired arrival time
        if (cap > 0) {
            earliestArr = 1000000;
            latestArr = -1;
            for (p = 0; p < C; p++) {
                sumx = 0;
                for (int j = 0; j < Stations; j++) {
                    sumx += xk[i][indexpt[p]][j];
                }
                if (sumx == 1) {
                    earliestArr = arrivals[indexpt[p]];
                    break;
                }
            }
            for (p = C - 1; p >= 0; p--) {
                sumx = 0;
                for (int j = 0; j < Stations; j++) {
                    sumx += xk[i][indexpt[p]][j];
                }
                if (sumx == 1) {
                    latestArr = arrivals[indexpt[p]];
                    break;
                }
            }

            // cout << " earliest " << earliestArr + d_time2 << ", latest " << latestArr-d_time1 << endl;

            if (BusPa.size() != 0) {
                Arr = findMedian(BusPa);
                if (Arr <= std::max(latestArr - d_time1, earliestArr)) {
                    Arr = std::max(latestArr - d_time1, earliestArr);
                    // cout << " too low \n";
                }
                else if (Arr >= std::min(earliestArr + d_time2, latestArr)) {
                    Arr = std::min(earliestArr + d_time2, latestArr);
                    // cout << " too high \n";
                }
            }
            else {
                Arr = Ak[i];
                // cout << " No passengers \n";
            }
            // cout << "Chosen arrival : " << Arr << endl;
        }
        else {
            Arr = Ak[i];
            // cout << "Chosen arrival : " << Arr << endl;
        }
        /*
        countcap = 0;
        p_0 = 0;
        p_new = 0;
        for (p = 0; p < C; p++) {
            sumx = 0;
            for (int j = 0; j < Stations; j++) sumx += xk[i][indexpt[p]][j];
            if (sumx == 1) {
                //cout << indexpt[p] << " -> " << arrivals[indexpt[p]] << endl;
                countcap++;
                p_0 = p_new;
                p_new = p;
                if (cap % 2 == 0 && countcap >= cap / 2) {
                    Arr = arrivals[indexpt[p]];
                    //cout << arrivals[indexpt[p]] << endl;
                    break;
                }
                else if (cap % 2 != 0 && countcap >= cap / 2+1) {
                    Arr = (arrivals[indexpt[p_new]] + arrivals[indexpt[p_0]]) / 2.0;
                    break;
                }
            }
        }
        if ((Arr - earliestArr) > d_time2)Arr = earliestArr + d_time2;
        if ((latestArr - Arr) > d_time1)Arr = latestArr - d_time1;
        //*/
        // Arr = earliestArr + d_time2;

        // Akk[p] += Dk[i];
        Dk[i] = Arr - Ak[i];

        // if (Dk[i] < 0) cout << " what " << endl;
        Ak[i] = Arr;
        // cout << " arrval time " << Ak[i] << endl;

        // COSTS --------------------------------------------------------------
        for (int j = 0; j < Stations; j++) {
            for (int k = 0; k < Stations; k++) {
                cost += c1 * yk[i][j][k] * (traveltimes[j][k] + delta);
            }

            for (p = 0; p < C; p++) {
                cost += xk[i][p][j] * (c1 * (double)tau + c2 * traveltimep[p][j]);
            }
        }

        for (p = 0; p < C; p++) {
            sumx = 0;
            for (int j = 0; j < Stations; j++) {
                sumx += xk[i][p][j];
            }
            if (sumx == 1) {
                if (arrivals[p] > Ak[i]) {
                    cost += c3 * abs(arrivals[p] - Ak[i]);
                }
                else {
                    cost += c3 * abs(arrivals[p] - Ak[i]);
                }
            }
        }

        for (p = 0; p < C; p++) {
            pindex[p] = 0;
        }
    }

    if (pa < C) {
        std::cout << "----------------- INFEASIBLE for capacity ----------------------\n " << std::endl;
        exit(0);
    }
}

// 2-opt improvement for the inital solution's routing
void twoOptImprovement(std::clock_t &start_time) {
    for (int i = 0; i < nBuses; i++) {

        // FILL IN ROUTE
        int j = 0;
        route.push_back(j);
        int n = 1;
        // std::cout << route[n - 1] << " ";
        while (j != N - 1) {
            for (int k = 1; k < Stations; k++) {
                if (yk[i][j][k] == 1) {
                    yk[i][j][k] = 0;
                    route.push_back(k);
                    n++;
                    j = k;
                    // std::cout << k << " ";
                    break;
                }
            }
        }
        // std::cout << "\n----- n: " << n << endl;

        // Number of 2-opt operations
        for (int p = 0; p < 3000; p++) {
            j = rand_station(rng_engine);
            while (j == N - 1) {
                j = rand_station(rng_engine);
            }
            int l = rand_station(rng_engine);
            while (l == j || l == N - 1) {
                l = rand_station(rng_engine);
            }
            index1 = giveIndex(route, j, n);
            if (index1 == n) {
                continue;
            }
            index2 = giveIndex(route, l, n);
            if (index2 == n) {
                continue;
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

            currentcost = traveltimes[route[start]][route[start + 1]] + delta + traveltimes[route[end_i]][route[nextindex]] + delta;
            ncost = traveltimes[route[start]][route[end_i]] + delta + traveltimes[route[start + 1]][route[nextindex]] + delta;

            // std::cout << " Current cost: " << currentcost << " New Cost: " << newcost << " \n";

            //-------------------------------------------------------------------- SA --------------------------------------------------------------------------
            dE = ncost - currentcost;
            if (dE < 0) {
                // std::cout << " Found better dE =" << dE << "   +++++++++++++++++++++++\n";
                // std::cout << " it : " << p << endl;
                cost += c1 * dE;
                Dk[i] -= dE;
                mid = (end_i - start) / 2;

                // 2opt
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
        last = (double)(clock() - start_time) / CLOCKS_PER_SEC;

        // std::cout << " NEW \n";
        j = 0;
        n = 1;
        // std::cout << j << " ";
        while (j != N - 1) {
            for (int k = 1; k < Stations; k++) {
                if (yk[i][j][k] == 1) {
                    n++;
                    j = k;
                    // std::cout << k << " ";
                    break;
                }
            }
        }
        // std::cout << "\n----- n: " << n << endl;
    }
}