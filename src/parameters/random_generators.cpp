#include "parameters/random_generators.h"
#include "parameters/hardcoded.h"
#include "parameters/ohers.h"

// Random number generator
std::default_random_engine rng_engine;

// Random distributions (default initialized, will be properly set by initializeRandomDistributions)
std::uniform_int_distribution<int> rand_passenger;
std::uniform_int_distribution<int> rand_station;
std::uniform_int_distribution<int> rand_percentage(1, 100); // This one doesn't depend on runtime values
std::uniform_int_distribution<int> rand_bus_weighted;
std::uniform_int_distribution<int> rand_destroy_count(3, 5); // This one doesn't depend on runtime values

// Initialize distributions after parameters are loaded
void initializeRandomDistributions() {
    rand_passenger = std::uniform_int_distribution<int>(0, C - 1);
    rand_station = std::uniform_int_distribution<int>(0, Stations - 1);
    rand_bus_weighted = std::uniform_int_distribution<int>(1, Np);
}
