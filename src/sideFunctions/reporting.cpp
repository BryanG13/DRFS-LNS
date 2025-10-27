// parameters
#include "parameters/hardcoded.h"
#include "parameters/ohers.h"
#include "parameters/optimization.h"

#include <fstream>

using std::endl;

// save instance as a formal txt file 
void saveInstance(){
    std::ofstream inst("Data/results/Instance_H_" + std::to_string(instance) + ".txt");
	inst << "---------- Weight factors of the objective function -------- " << endl << endl;
	inst << "c1: " << c1 << " \t (For travel-time of the buses)" << endl;
	inst << "c2: " << c2 << " \t (For walking time of the passengers)" << endl;
	inst << "c3: " << c3 << " \t (For the absolute difference in desired arrival time and actual arrival time of the passengers)" << endl;
	inst << "Number of buses: " << nBuses << endl;
	inst << "Number of mandatory bus stops: " << N << endl;
	inst << "Number optional bus stops per cluster: " << M << " \n --> One cluster between each mandatory stop: " + std::to_string((N - 1) * M) + " optional stops in total" << endl;
	inst << "Total number of bus stops: " << Stations << endl;
	inst << "Number of passenger requests: " << C << endl;
	inst << "Bus capacity: " << bCapacity << " passengers" << endl;
	inst << "Acceleration and deceleration time: " << delta << " seconds" << endl;
	inst << "Dwell time per passenger at a bus stop: " << tau << " seconds" << endl;
	inst << "Maximum walking time for any passenger: " << d << " seconds" << endl;
	inst << "Maximum amount of time a passenger can arrive too early: " << d_time1 << " seconds" << endl;
	inst << "Maximum amount of time a passenger can arrive too late: " << d_time2 << " seconds" << endl;

	inst << endl << "Desired arrival times of the passengers in seconds: " << endl;
	int i = 0;
	while (i < C) {
		inst << "p_" << i + 1 << ": \t" << arrivals[i] << endl;
		i++;
	}

	inst << endl << "Walking time between passengers and bus stops in seconds: " << "\npassengers correspond with the rows, bus stops correspond with the columns \nthe mandatory stops are listed first, then the optional stops are listed" << endl;
	for (int i = 0; i < C; i++) {
		for (int j = 0; j < Stations; j++) {
			inst << int(traveltimep[i][j]) << "\t";
		}
		inst << endl;
	}

	inst << endl << "Travel time between bus stops in seconds: " << endl;
	for (int i = 0; i < Stations; i++) {
		for (int j = 0; j < Stations; j++) {
			inst << int(traveltimes[i][j]) << "\t";
		}
		inst << endl;
	}
	inst.close();
}


// write solution to files
void writeSolution(){
	std::ofstream txt_rt("runTime.txt");
	std::ofstream txt_pg("progress.txt");
	for (int i = 0; i < best_srt.size(); i++) {
		txt_rt << best_srt[i] << endl;
		txt_pg << best_prog[i] << endl;
	}
	txt_rt.close();
	txt_pg.close();
	std::ofstream txt_xsol("Data/results/xsol_" + std::to_string(instance) + ".txt");
	std::ofstream txt_ysol("Data/results/ysol_" + std::to_string(instance) + ".txt");
	std::ofstream txt_Dsol("Data/results/Dsol_" + std::to_string(instance) + ".txt");
	std::ofstream txt_Asol("Data/results/Asol_" + std::to_string(instance) + ".txt");

	for (int i = 0; i < nBuses; i++) {
		txt_Dsol << Dbest[i] << endl;
		txt_xsol << "Bus " << i << endl;
		txt_ysol << "Bus " << i << endl;
		for (int j = 0; j < Stations; j++) {
			for (int k = 0; k < Stations; k++) {
				txt_ysol << ybest[i][j][k] << " ";
			}
			for (int p = 0; p < C; p++) {
				txt_xsol << xbest[i][p][j] << " ";
			}
			txt_xsol << endl;
			txt_ysol << endl;
		}
		txt_xsol << "end" << endl;
		txt_ysol << "end" << endl;
	}
	double Akk[C];
	for (int p = 0; p < C; p++) {
		for (int i = 0; i < nBuses; i++) {
			for (int j = 0; j < Stations; j++) {
				if (xbest[i][p][j] > 0) {
					//txt_Asol << Abest[i] << endl;
					Akk[p] = Abest[i];
				}
			}
		}
	}

	for (int p = 0; p < C; p++) {
		//cout << Akk[p] - arrivals[p] << endl;
		txt_Asol << Akk[p] << endl;
	}


	txt_xsol.close();
	txt_ysol.close();
	txt_Asol.close();
	txt_Dsol.close();

	std::ofstream txt_visits("Data/results/visits_" + std::to_string(instance) + ".txt");
	int j = 0;
	double travel = 0;
	for (int i = 0; i < nBuses; i++) {
		j = 0;
		travel = Dbest[i];
		txt_visits << "Bus " << i + 1 << endl;
		while (j != N - 1) {
			for (int k = 0; k < Stations; k++) {
				if (yk[i][j][k] == 1) {
					txt_visits << j << " " << travel << endl;
					travel += (traveltimes[j][k] + delta);
					for (int p = 0; p < C; p++) {
						travel += xbest[i][p][j] * tau;
					}
					j = k;
				}
			}
		}
		txt_visits << j << " " << travel << endl;
		txt_visits << "end" << endl;
	}
	txt_visits.close();


	std::ofstream txt_walking("Data/results/walking_" + std::to_string(instance) + ".txt");
	for (int p = 0; p < C; p++) {
		for (int j = 0; j < Stations; j++) {
			for (int i = 0; i < nBuses; i++) {
				if (xbest[i][p][j] == 1) {
					txt_walking << j << " " << traveltimep[p][j] << endl;
				}
			}
		}
	}
	txt_walking.close();
}
