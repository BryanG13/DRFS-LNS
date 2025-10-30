

int instance = 14; // Instance number for indexing purposes

int nRUN = 10; // Number of runs

// WEIGHT FACTORS--------------------------------------------------------------------------
float c1 = 0.25f;       // (For travel-time of the buses)
float c2 = 0.35f;       // (For walking time of the passengers)
float c3 = 1 - c1 - c2; // (For the absolute difference in desired arrival time and actual arrival time of the passengers)

// Define parameters-----------------------------------------------------------------------
const int nBuses = 5;                 // number of buses available
const int N = 10;                     // number of mandatory stations
const int M = 3;                      // number of stations in cluster
const int Stations = (N - 1) * M + N; // amount of Stations
const int C = 40;                     // number of clients in opt horizon

const int bCapacity = 15;          // Bus capcity
const float pspeed = 1.0f;         // passengers speed in meter per second
const float bspeed = 40.0f / 3.6f; // bus speed in m/s
const int delta = 30;              // acceleration and deceleration time  in seconds
const int tau = 5;                 // dwell time coeficient in seconds
const int d = 20 * 60;             // threshold of individual walking time in seconds
const int d_time1 = 15 * 60;       // Maximum amount of time a passenger can arrive too early
const int d_time2 = 5 * 60;        // Maximum amount of time a passenger can arrive too late

int f_IT = 5000;        // max iteraions for he LNS cycles
int ndestroy = 3;       // number of passengers to remove
int secondS = 100 - 25; // Second station selection threshold (percentage)
int thirdS = 5;         // Third station selection threshold (percentage)