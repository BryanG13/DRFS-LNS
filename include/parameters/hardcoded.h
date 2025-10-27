#ifndef HARDCODED_H
#define HARDCODED_H

extern int instance; // Instance number for indexing purposes

extern int nRUN; // Number of runs 

//WEIGHT FACTORS--------------------------------------------------------------------------
extern float c1; // (For travel-time of the buses)
extern float c2; // (For walking time of the passengers)
extern float c3; // (For the absolute difference in desired arrival time and actual arrival time of the passengers)

//Define parameters-----------------------------------------------------------------------
constexpr int nBuses = 5; // number of buses available
constexpr int N = 10; // number of mandatory stations
constexpr int M = 3; // number of stations in cluster
constexpr int Stations = (N - 1) * M + N; // amount of Stations
constexpr int C = 40; // number of clients in opt horizon

constexpr int bCapacity = 15; // Bus capacity
constexpr float pspeed = 1.0f; // passengers speed in meter per second
constexpr float bspeed = 40.0f / 3.6f; // bus speed in m/s
constexpr int delta = 30; // acceleration and deceleration time  in seconds
constexpr int tau = 5; //dwell time coefficient in seconds
constexpr int d = 20 * 60; // threshold of individual walking time in seconds
constexpr int d_time1 = 15 * 60; // Maximum amount of time a passenger can arrive too early
constexpr int d_time2 = 5 * 60; // Maximum amount of time a passenger can arrive too late

extern int f_IT; // max iteraions for the LNS cycles 
extern int ndestroy; // number of passengers to remove
extern int secondS; // Second station selection threshold (percentage)
extern int thirdS; // Third station selection threshold (percentage)

#endif // HARDCODED_H
