#include <iostream>
#include <fstream>
//#include "Datagen.h"
#include <stdlib.h> 
#include <random>
#include <time.h>
#include <cmath>

// include the side funcion
#include "sideFunctions/sorting.h"
#include "sideFunctions/reporting.h"

// parameters
#include "parameters/hardcoded.h"
#include "parameters/ohers.h"
#include "parameters/optimization.h"
#include "parameters/random_generators.h"

using namespace std;

int main() {

	int p, j, k, l, c, i, n; // indices 

	//************************************************* PRE PROCESSING ********************************************************
	// small pre processing of parameters 
	processingParameters();
	saveInstance();

	// Initialize random distributions after parameters are loaded
	initializeRandomDistributions();

	// initialize a solution raw pointer 
	initilalizeX();

	//std::cout << "cost: " << cost << endl;

	ofstream txt_runs("Data/results/Runs_" + to_string(instance) + ".txt");

	for(int run = 0;run < nRUN; run++){
		//************************************************* INITIALIZE ********************************************************
		// time the heuristic 
		clock_t start_time;
		start_time = clock();

		//initialize cost 
		cost = 0;

		// initialize indices
		cc = 0;
		n = 0;
		for (p = 0; p < C; p++) {
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

		// for each bus avaialable  
		for (i = 0; i < nBuses; i++) {
			//cout << "Bus " << i + 1 << endl;
			cap = 0;
			BusPa.clear();

			//intialize vector y
			for (j = 0; j < Stations; j++) {
				for (int k = 0; k < Stations; k++) {
					yk[i][j][k] = 0;
				}
			}

			//intialize vector x
			for (p = 0; p < C; p++) {
				for (k = 0; k < Stations; k++) {
					xk[i][p][k] = 0;
				}
			}

			//fill in the x variable
			for (p = pa; p < C; p++) {
				if (cap > 0)timewindow = abs(firsttw - arrivals[indexpt[p]]);
				else {
					firsttw = arrivals[indexpt[p]];
					timewindow = 0;
				}
				//std::cout << "passenger " << indexpt[p] << " time window is " << timewindow  << " <= " << (d_time1 + d_time2)  << endl;
				//cout << cap << "<" << bCapacity << endl;
				//cout << pa << "<" << C << endl;
				if (cap < bCapacity && pa < C && (int)timewindow <= d_time1 + d_time2) {
					minpsi = closestPS[indexpt[p]][0];
					if (traveltimep[indexpt[p]][minpsi] <= d) {
						xk[i][indexpt[p]][minpsi] = 1;
						BusPa.push_back(arrivals[indexpt[p]]);
						//cout << "Chosen \n";
						cap++;
						pindex[pa] = minpsi;
						pa++;
						//cout << "passenger " << indexpt[p] << " arrival " << arrivals[indexpt[p]] << endl;
					}
					else {
						std::cout << "INFEASIBLE for walking \n";
						cout << "passenger " << indexpt[p]+1 << " has a minimum walking time of " << traveltimep[indexpt[p]][minpsi] << endl;
						exit(0);
					}
				}
			}

			// all mandatory Staions
			for (k = 0; k < N - 1; k++) {
				yk[i][k][k + 1] = 1;
			}

			p = 0;
			//fill in the y variable 
			for (const auto& p : pindex) {
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
					if (already) continue;
					//cout << "Optional: " << p << endl;
					for (int j = 0; j < N; j++) {
						if (traveltimes[j][p] < minDist && j != p) {
							minDist = traveltimes[j][p];
							mini = j;
						}
					}
					if ((p - N) / M + 1 <= mini) mini--;
					//cout <<"Mandatory " << mini << endl;


					if (yk[i][mini][mini + 1] != 0) {
						yk[i][mini][mini + 1] = 0;
						yk[i][mini][p] = 1;
						yk[i][p][mini + 1] = 1;
					}
					else {
						int gotcha = -1;
						for (const auto& pp : pindex) {
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
			for (j = 0; j < Stations; j++) {
				for (k = 0; k < Stations; k++) {
					Ak[i] += (traveltimes[j][k] + delta) * yk[i][j][k];
				}
				for (c = 0; c < C; c++) {
					Ak[i] += (double)tau * xk[i][c][j];
				}
			}

			//Calculate earliest desired arrival time
			if (cap > 0) {
				earliestArr = 1000000;
				latestArr = -1;
				for (p = 0; p < C; p++) {
					sumx = 0;
					for (int j = 0; j < Stations; j++) sumx += xk[i][indexpt[p]][j];
					if (sumx == 1) {
						earliestArr = arrivals[indexpt[p]];
						break;
					}
				}
				for (p = C - 1; p >= 0; p--) {
					sumx = 0;
					for (int j = 0; j < Stations; j++) sumx += xk[i][indexpt[p]][j];
					if (sumx == 1) {
						latestArr = arrivals[indexpt[p]];
						break;
					}
				}

				//cout << " earliest " << earliestArr + d_time2 << ", latest " << latestArr-d_time1 << endl;

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
					Arr = Ak[i];
					//cout << " No passengers \n";
				}
				//cout << "Chosen arrival : " << Arr << endl;
			}
			else {
				Arr = Ak[i];
				//cout << "Chosen arrival : " << Arr << endl;
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
			//Arr = earliestArr + d_time2;


			//Akk[p] += Dk[i];
			Dk[i] = Arr - Ak[i];

			//if (Dk[i] < 0) cout << " what " << endl;
			Ak[i] = Arr;
			//cout << " arrval time " << Ak[i] << endl;


			// COSTS --------------------------------------------------------------
			for (j = 0; j < Stations; j++) {
				for (k = 0; k < Stations; k++) {
					cost += c1 * yk[i][j][k] * (traveltimes[j][k] + delta);
				}

				for (p = 0; p < C; p++) {
					cost += xk[i][p][j] * (c1 * (double)tau + c2 * traveltimep[p][j]);
				}
			}

			for (p = 0; p < C; p++) {
				sumx = 0;
				for (j = 0; j < Stations; j++) sumx += xk[i][p][j];
				if (sumx == 1) {
					if (arrivals[p] > Ak[i]) cost += c3 * abs(arrivals[p] - Ak[i]);
					else cost +=  c3 * abs(arrivals[p] - Ak[i]);
				}
			}

			for (p = 0; p < C; p++) {
				pindex[p] = 0;
			}
		}

		if (pa < C) {
			std::cout << "----------------- INFEASIBLE for capacity ---------------------- " << endl << endl;
			exit(0);
		}
		/*
		for (int i = 0; i < nBuses; i++) {
			for (int p = 0; p < C; p++) {
				for (int j = 0; j < Stations; j++) {
					cout << xk[i][p][j] << ' ';
				}
				cout << endl;
			}
			cout << endl;
		}

		exit(0);
		*/

		//*// simple 2-opt to improve routing
		for (i = 0; i < nBuses; i++) {

			//FILL IN ROUTE
			j = 0;
			route.push_back(j);
			n = 1;
			//std::cout << route[n - 1] << " ";
			while (j != N - 1) {
				for (k = 1; k < Stations; k++) {
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

			//amount of opt operations
			for (p = 0; p < 3000; p++) {
				j = rand_station(rng_engine);
				while (j == N - 1) j = rand_station(rng_engine);
				l = rand_station(rng_engine);
				while (l == j || l == N - 1) l = rand_station(rng_engine);
				index1 = giveIndex(route, j, n);
				if (index1 == n) continue;
				index2 = giveIndex(route, l, n);
				if (index2 == n) continue;

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

				//std::cout << " Current cost: " << currentcost << " New Cost: " << newcost << " \n";

				//--------------------------------------------------------------------SA --------------------------------------------------------------------------
				dE = ncost - currentcost;
				if (dE < 0) {
					//std::cout << " Found better dE =" << dE << "   +++++++++++++++++++++++\n";
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
			last= (double)(clock() - start_time) / CLOCKS_PER_SEC;

			//std::cout << " NEW \n";
			j = 0;
			n = 1;
			//std::cout << j << " ";
			while (j != N - 1) {
				for (k = 1; k < Stations; k++) {
					if (yk[i][j][k] == 1) {
						n++;
						j = k;
						//std::cout << k << " ";
						break;
					}
				}
			}
			//std::cout << "\n----- n: " << n << endl;
		}


		//std::cout << " Initial cost: " << cost << endl;


		//********************************************************* |||||||||||     HEURISTIC     ||||||||||||||| **********************************************************


		it = 0;
		nextcost = 0;
		/*
		for (int p = 0; p < C; p++) {
			for (int i = 0; i < nBuses; i++) {
				int sumx = 0;
				for (int j = 0; j < Stations; j++) {
					sumx += xk[i][p][j];
				}
				rebuild[i][p] = sumx;
				//cout << rebuild[i][p] << " ";
			}
			//cout << endl;
		}
		//cout << endl;
		*/

		infeas = false;
		
		while (it - bestit <= f_IT) {
			ndestroy = rand_destroy_count(rng_engine);
			//std::cout << "Iteration:	" << it + 1 << " ---------------" << endl;

			// Initialize xsol and ysol
			for (i = 0; i < nBuses; i++) {
				for (j = 0; j < Stations; j++) {
					for (k = 0; k < Stations; k++) ysol[i][j][k] = 0;
					for (p = 0; p < C; p++) xsol[i][p][j] = xk[i][p][j];
				}
				// calculate the used capacity and time window
				bCapi[i] = 0;

				earlyTW[i] = 1000000000;
				lateTW[i] = -1;
				for (p = 0; p < C; p++) {
					for (j = 0; j < Stations; j++) {
						if (xsol[i][p][j] == 1) {
							bCapi[i]++;
							if (arrivals[p] > lateTW[i])lateTW[i] = arrivals[p];
							if (arrivals[p] < earlyTW[i])earlyTW[i] = arrivals[p];
							break;
						}
					}
				}
			}

			/*
			j = 0;
			cout << j << " ";
			while (j != N - 1) {
				for (k = 1; k < Stations; k++) {
					if (ysol[1][j][k] == 1) {
						j = k;
						cout << k << " ";
						break;
					}
				}
			}
			cout << endl;
			*/

			//Destroyed is the array keeping track of the passangers that are removed, at each iteration the array is reset 
			for (p = 0; p < C; p++) destroyed[p] = -1;



			//**************************************** || DESTROY (and assign new bus for passengers) || *************************
			for (l = 0; l < ndestroy;) {
				di = rand_passenger(rng_engine); // passanger to remove is chosen randomly

				//only if the chosen destroy passenger hasnt been destroyed yet
				if (destroyed[di] == -1) {

					//reduced the used capacity of the bus i
					pbi = pst = -1;
					for (i = 0; i < nBuses; i++) {
						for (j = 0; j < Stations; j++) {
							if (xsol[i][di][j] == 1) {
								pst = j; //previous station 
								pbi = i; // previous bus (of destroyed passenger)
								break;
							}
						}
						if (pbi == i) break;
					}
					if (pbi == -1) {
						std::cout << " oops passenger " << di << " not found " << endl << " ------------ INFEASIBLE for arrival times  ------------";
						exit(0);
					}

					//determine the bus to assign passenger again

					//rank buses according to arrival times of the passengers it is carrying
					for (i = 0; i < nBuses; i++) {
						rank_buses[i] = i;
						value_buses[i] = 0;
						n = 0;
						for (p = 0; p < C; p++) {
							sumx = 0;
							for (j = 0; j < Stations; j++) sumx += xk[i][p][j];
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
					for (i = nBuses; i >= 1; i--) {
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

			//Set nexcost back to 0
			nextcost = 0;

			//std::cout << "destroyed \n";
			//***************************************** || REBUILD FUNCTION || ****************************************************
			for (i = 0; i < nBuses; i++) {

				//cout << " Bus " << i + 1 << endl;

				//ADJUST x  VARIABLES *************************************************
				for (p = 0; p < C; p++) {
					//for the passengers that were taken out of the solution and are assigned to this bus i
					if (destroyed[p] == i) {
						//adjust in the x variable TO NEW STATION (account for walking constraint)--------------- 

						if (traveltimep[p][closestPS[p][0]] > d) {
							std::cout << " Infeasible solution, for walking times " << endl;
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
							for (c = 0; c < C; c++) {
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

				j = 0;
				imand = 0;

				// not feasible, forgets stations !!!! ----------------------------------------------------------------------------------------------------
				while (j != N - 1) {
					//cout << j << endl;
					// determine the last mandatory station visited 
					if (j < N - 1) imand = j;

					//cout << j << " Has NO successor  "  ;
					firstmin = 100000000, firstminj = -1;
					for (k = N + imand * M; k < N + (imand + 1) * M; k++) { // first check the optional stations 
						//check if node is in chosen subset
						inGraph = false;
						for (p = 0; p < C; p++) {
							if (xsol[i][p][k] > 0) {
								inGraph = true;
								break;
							}
						}

						//check if node is already a successor
						inSuccessor = false;
						for (l = 0; l < Stations; l++) {
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
						for (k = 1; k < N; k++) { //then check the mandatory ones

							//check if node is already a successor
							inSuccessor = false;
							for (l = 0; l < Stations; l++) {
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
						for (k = std::max(N + (imand - 1) * M, N); k < std::min(N + (imand + 2) * M, Stations); k++) { // first check the optional stations 
						//check if node is in chosen subset
							inGraph = false;
							for (p = 0; p < C; p++) {
								if (xsol[i][p][k] == 1) {
									inGraph = true;
									break;
								}
							}

							//check if node is already a successor
							inSuccessor = false;
							for (l = 0; l < Stations; l++) {
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

				/*// simple 2-opt to improve routing -----
				//FILL IN ROUTE
				j = 0;
				route.push_back(j);
				n = 1;
				//cout << route[n-1] << " ";
				while(j!=N-1) {
					for (k = 1; k < Stations; k++) {
						if (ysol[i][j][k] == 1) {
							ysol[i][j][k] = 0;
							route.push_back(k);
							n++;
							j = k;
							//cout << k << " ";
							break;
						}
					}
				}
				//cout << "\n----- n: " << n << endl;

				//amount of opt operations
				for (p = 0; p < 500; p++) {
					j = rand_station(rng_engine);
					while (j == N - 1) j = rand_station(rng_engine);
					l = rand_station(rng_engine);
					while (l == j || l == N-1) l = rand_station(rng_engine);
					index1 = giveIndex(route, j, n);
					if (index1 == n) continue;
					index2 = giveIndex(route, l, n);
					if (index2 == n) continue;

					if (index1 > index2) {
						start = index2;
						end_i = index1;
					}
					else {
						start = index1;
						end_i = index2;
					}

					nextindex = end_i + 1;

					currentcost = traveltimes[route[start]][route[start + 1]] + traveltimes[route[end]][route[nextindex]];
					ncost = traveltimes[route[start]][route[end]] + traveltimes[route[start + 1]][route[nextindex]];

					dE = ncost - currentcost;
					if (dE < 0) {


						//std::cout << " Found better dE =" << dE<< "   +++++++++++++++++++++++\n";
						cout << " Bus : " << i+1 << " |" << route[start] << ", " << route[start + 1] << " -  " << route[end] << ", " << route[nextindex] << endl;

						//cout << " it : " << p << endl;
						mid = (end_i - start) / 2;

						//2opt
						for (cc = 1; cc <= mid; cc++) {
							temp = route[start + cc];
							route[start + cc] = route[end_i + 1 - cc];
							route[end_i + 1 - cc] = temp;
						}

						for (int tt = 0; tt < n; tt++) {
							cout << route[tt] << " ";
						}
						cout << endl;
					}
				}

				for (j = 0; j < n-1; j++) {
					ysol[i][route[j]][route[j + 1]] = 1;
				}

				route.clear();
				//*/

				//Calculate earliest desired arrival time
				earliestArr = 1000000;
				latestArr = -1;
				for (p = 0; p < C; p++) {
					sumx = 0;
					for (j = 0; j < Stations; j++) sumx += xsol[i][indexpt[p]][j];
					if (sumx == 1) {
						earliestArr = arrivals[indexpt[p]];
						break;
					}
				}
				for (p = C - 1; p >= 0; p--) {
					sumx = 0;
					for (j = 0; j < Stations; j++) sumx += xsol[i][indexpt[p]][j];
					if (sumx == 1) {
						latestArr = arrivals[indexpt[p]];
						break;
					}
				}
				/*
				countcap = 0;
				p_0 = 0;
				p_new = 0;
				for (p = 0; p < C; p++) {
					sumx = 0;
					for (int j = 0; j < Stations; j++) sumx += xsol[i][indexpt[p]][j];
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
						else if (cap % 2 != 0 && countcap >= cap / 2 + 1) {
							Arr = (arrivals[indexpt[p_new]] + arrivals[indexpt[p_0]]) / 2.0;
							break;
						}
					}
				}
				if ((Arr - earliestArr) > d_time2)Arr = earliestArr + d_time2;
				if ((latestArr - Arr) > d_time1)Arr = latestArr - d_time1;
				Arr = earliestArr + d_time2;
				//*/

				// fill in the A VARIABLE  ********************************************
				BusPa.clear();
				Asol[i] = 0;
				//Dsol[i] = 0;
				for (j = 0; j < Stations; j++) {
					for (k = 0; k < Stations; k++) {
						Asol[i] += (traveltimes[j][k] + delta) * ysol[i][j][k];
					}
					for (c = 0; c < C; c++) {
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
					std::cout << " Infeasible solution, for arrival times " << endl;
					infeas = true;
					break;
				}

				// COSTS --------------------------------------------------------------
				for (j = 0; j < Stations; j++) {
					for (k = 0; k < Stations; k++) {
						nextcost += c1 * ysol[i][j][k] * (traveltimes[j][k] + delta);
					}

					for (p = 0; p < C; p++) {
						nextcost += xsol[i][p][j] * (c1 * (double)tau + c2 * traveltimep[p][j]);
					}
				}

				for (p = 0; p < C; p++) {
					sumx = 0;
					for (j = 0; j < Stations; j++) sumx += xsol[i][p][j];
					if (sumx > 0) {
						 nextcost += c3 * fabs(arrivals[p] - Asol[i]);
					}
				}
			}

			//if one of the buses is infeasible, go to next proposal for a silution
			if (infeas) {
				it++;
				infeas = false;
				continue;
			}


			// ACCEPTANCE CRITERIA ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

			if ((int)(nextcost*100) < (int)(cost*100)) {
				bestit = it;
				cost = nextcost;
				last= (double)(clock() - start_time) / CLOCKS_PER_SEC;
				rt.push_back(last);
				prog.push_back(cost);
				// assign new solutions
				for (i = 0; i < nBuses; i++) {
					cap = 0;
					Ak[i] = Asol[i];
					Dk[i] = Dsol[i];
					for (j = 0; j < Stations; j++) {
						for (k = 0; k < Stations; k++) yk[i][j][k] = ysol[i][j][k];

						for (p = 0; p < C; p++) {
							xk[i][p][j] = xsol[i][p][j];
							//if (xk[i][p][j] == 1) cap++;
						}
					}
					//std::cout << " Bus " << i + 1 << " cap: " << cap << " max : " << bCapacity << endl;
				}
			}

			it++;
		}
		//std::cout << "END ---------------------------------------------------------------- \n it: " << it << "best it: " << bestit<< endl;

		//*// simple 2-opt to improve routing
			//SA PARAMETERS -----------------------------------------------------------------------------
			//amount of nearest neighbors 
		maxdist = -1;
		for (i = 0; i < n; i++) {
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

		for (i = 0; i < nBuses; i++) {

			//FILL IN ROUTE
			j = 0;
			route.push_back(j);
			n = 1;
			//std::cout << route[n - 1] << " ";
			while (j != N - 1) {
				for (k = 1; k < Stations; k++) {
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
			for (p = 0; p < 5000; p++) {
				j = rand_station(rng_engine);
				index1 = giveIndex(route, j, n);
				while (j == N - 1 || index1 == n) {
					j = rand_station(rng_engine);
					index1 = giveIndex(route, j, n);
				}

				l = closestS[j][NEXT(rng_engine)];
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
		
		//*/
		elapsed_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		rt.push_back(elapsed_time);
		prog.push_back(cost);
		std::cout << " Run number: " << run + 1 << "   ------------------------------------------------------- " << endl;
		std::cout << "Computational time: " << elapsed_time << " seconds " << endl;

		//std::cout << "-------- || Solution || -------- " << endl << endl;

		std::cout << "Cost: " << cost << endl << endl;

		txt_runs << cost << "\t" << elapsed_time << endl;

		if ((int)(best_cost*100) == (int)(cost*100) || run == 0) {
			if (best_rt > elapsed_time) {
				best_rt = elapsed_time;
				bestlast = last;
			}
			//cout << "better time " << endl;
		}
		if (best_cost > cost) {
			best_prog = prog;
			best_srt = rt;
			best_cost = cost;

			for (i = 0; i < nBuses; i++) {
				Abest[i] = Ak[i];
				Dbest[i] = Dk[i];
				for (j = 0; j < Stations; j++) {
					for (k = 0; k < Stations; k++) ybest[i][j][k] = yk[i][j][k];
					for (p = 0; p < C; p++) {
						xbest[i][p][j] = xk[i][p][j];
					}
				}
			}
		}
		rt.clear();
		prog.clear();
	}

	// some printing 
	std::cout << endl << "+++++++++++++++++++++++++++++ \n Best solution for inst. " << instance << ":  \n cost: " << best_cost << "\n run-time: " << best_rt << "s \n last improvement: "<<bestlast<<"s \n+++++++++++++++++++++++++++++\n";
	txt_runs.close();
	
	// Write to file 
	writeSolution();

	//remove memory
	deletePointers();

}