#include "parameters/hardcoded.h"
#include "sideFunctions/sorting.h"

#include <vector>
#include <iostream>
#include <fstream>

int estbus; // Estimated passengers per bus
int Np = 0; //Cumulative sum counter (used to build the rp array)
int rp[nBuses + 1]; // Rank/Priority boundaries array (used for bus selection)

std::vector<double> BusPa; // arrival times of passengers in a bus 

// location coordinates
double passengers[C][2]; // passenger locations
double mandatory[N][2]; //mandatory stop locations
double optional[(N - 1) * M][2]; // optional stop locations

// Arrival times of the passengers 
double arrivals[C];

// Travel times
double traveltimep[C][Stations]; // travel times of people between passangers and stations
double traveltimes[Stations][Stations];// travel times of buses between stations

// Sets 
int I[nBuses]; // set for buses 
int J[Stations]; // set for stops
int F[N]; // set for mandatory stops
int O[(N - 1) * M]; // set for optional stops
int P[C]; // set for passenger requests

// Cloesest stops 
int closestS[Stations][Stations]; // closest stops to on another
int closestPS[C][Stations]; // closes stops to clients 

// temp
int indexpt[C]; // passenger indices sorted by arrival time
double temparrivals[C]; // temporary copy of arrival times for sorting

// 2 opt
std::vector<int> route; // route sequence for 2-opt optimization
int cc; // counter for 2-opt swaps
int index1, index2, start, end_i, nextindex, mid, temp; // indices for 2-opt operations
int imand; // index of last mandatory station visited


// processing of the parameters: reading in data, calculating some useful metrics as input for optimization
void processingParameters(){
    if (C % nBuses == 0) estbus = C / nBuses;
	else estbus = C / nBuses + 1;

    for (int i = nBuses; i >= 0; i--) {
		Np += nBuses - i;
		rp[i] = Np;
	}

    // reading in 
    std::ifstream filep("Data/input/passengers.txt");//Passengers
	int i = 0;
	while (i < C) {
		filep >> passengers[i][0] >> passengers[i][1]; // extracts 2 floating point values seperated by whitespace
		i++;
	}

    std::ifstream filem("Data/input/mandatory.txt");
	i = 0;
	while (i < N) {
		filem >> mandatory[i][0] >> mandatory[i][1]; // extracts 2 floating point values seperated by whitespace
		i++;
	}

    std::ifstream fileo("Data/input/optional.txt");
	i = 0;
	while (i < (N - 1) * M) {
		fileo >> optional[i][0] >> optional[i][1]; // extracts 2 floating point values seperated by whitespace
		i++;
	}

    std::ifstream filea("Data/input/arrivals.txt");
	i = 0;
	while (i < C) {
		filea >> arrivals[i];
		i++;
	}

    //calculate travel time using manhattan distance
    for (int i = 0; i < C; i++) {
		for (int j = 0; j < N; j++) {
			traveltimep[i][j] = (abs(passengers[i][0] - mandatory[j][0]) + abs(passengers[i][1] - mandatory[j][1])) * 1000 / pspeed;
		}

		for (int j = N; j < Stations; j++) {
			traveltimep[i][j] = (abs(passengers[i][0] - optional[j - N][0]) + abs(passengers[i][1] - optional[j - N][1])) * 1000 / pspeed;
		}
	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			traveltimes[i][j] = (abs(mandatory[i][0] - mandatory[j][0]) + abs(mandatory[i][1] - mandatory[j][1])) * 1000 / bspeed;
		}
		for (int j = N; j < Stations; j++) {
			traveltimes[i][j] = (abs(mandatory[i][0] - optional[j - N][0]) + abs(mandatory[i][1] - optional[j - N][1])) * 1000 / bspeed;
		}
	}

	for (int i = N; i < Stations; i++) {
		for (int j = 0; j < N; j++) {
			traveltimes[i][j] = (abs(optional[i - N][0] - mandatory[j][0]) + abs(optional[i - N][1] - mandatory[j][1])) * 1000 / bspeed;
		}
		for (int j = N; j < Stations; j++) {
			traveltimes[i][j] = (abs(optional[i - N][0] - optional[j - N][0]) + abs(optional[i - N][1] - optional[j - N][1])) * 1000 / bspeed;
		}
	}

	// sets 
	for (i = 0; i < nBuses; i++) {
		I[i] = i;
	}
	for (i = 0; i < Stations; i++) {
		J[i] = i;
	}
	for (i = 0; i < N; i++) {
		F[i] = i;
	}
	for (i = N; i < Stations; i++) {
		O[i - N] = i;
	}
	for (i = 0; i < C; i++) {
		P[i] = i;
	}

	// Closests to stops arrays 
	double tempdist[Stations]; // temporary travel times for stops
	int index[Stations]; // index for stops
	//std::cout << "Stops: \n";
	for (int i = 0; i < Stations; i++) {
		//initialize and copy travel times of stops to temp array
		for (int j = 0; j < Stations; j++) {
			if (j != i) tempdist[j] = traveltimes[i][j];
			else tempdist[j] = 100000;
			index[j] = j;
		}

		//sort according to dist
		quickSort(index, tempdist, 0, Stations - 1);

		//keep track of best neighbors 
		for (int l = 0; l < Stations; l++) {
			closestS[i][l] = index[l]; //index
			//std::cout << index[l] << " ";
		}
		//std::cout << " \n";
	}

	int Index2[C]; // index for clients 
	//std::cout << "Passengers: \n";
	for (int p = 0; p < C; p++) {
		//initialize and copy distance of cities to temp array
		for (int j = 0; j < Stations; j++) {
			tempdist[j] = traveltimep[p][j];
			index[j] = j;
		}
		//sort according to dist
		quickSort(index, tempdist, 0, Stations - 1);

		//keep track of best neighbors 
		for (int j = 0; j < Stations; j++) {
			closestPS[p][j] = index[j];
			//std::cout << index[j] << " ";
		}
	}

	// intitialize 
	for (int p = 0; p < C; p++) {
		indexpt[p] = p;
		temparrivals[p] = arrivals[p];
	}
	//sort passengers by arrival time
	quickSort(indexpt, temparrivals, 0, C - 1);
}