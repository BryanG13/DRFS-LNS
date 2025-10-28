# DRFS-LNS: Demand-Responsive Flexible Services with Large Neighborhood Search

A C++ implementation of a Large Neighborhood Search (LNS) metaheuristic for solving bus routing optimization problems with flexible demand-responsive services.

## Overview

This project implements an optimization algorithm for bus routing that:
- Assigns passengers to buses while respecting capacity constraints
- Determines optimal station sequences for each bus route
- Minimizes total cost considering bus travel time, passenger walking time, and arrival time deviations
- Uses a destroy-and-repair heuristic combined with 2-opt local search and Simulated Annealing

## Problem Description

The algorithm solves a feeder bus routing problem with:
- **Mandatory stations**: Fixed stops that must be visited in sequence
- **Optional stations**: Additional stops that can be selectively included
- **Time windows**: Constraints on passenger desired arrival times
- **Capacity constraints**: Maximum passengers per bus
- **Walking distance limits**: Maximum distance passengers can walk to stations

### Objective Function

The cost function minimizes:
```
Cost = c1 × (bus travel time + dwell time) 
     + c2 × (passenger walking time)
     + c3 × (deviation from desired arrival time)
```

## Folder Structure

```
DRFS-LNS/
├── CMakeLists.txt              # CMake build configuration
├── LICENSE                     # MIT License
├── README.md                   # This file
├── .vscode/                    # VS Code configuration
│   ├── launch.json             # Debugger settings
│   └── tasks.json              # Build tasks
├── build/                      # Build output directory (generated)
├── Data/                       # Input/output data files
│   ├── instances/              # Problem instances
│   └── results/                # Solution outputs
├── include/                    # Header files
│   ├── parameters/             # Parameter declarations
│   │   ├── hardcoded.h         # Problem constants
│   │   ├── optimization.h      # Decision variables
│   │   ├── others.h            # Auxiliary variables
│   │   └── random_generators.h # Random number generators
│   └── sideFunctions/          # Utility function headers
│       ├── reporting.h         # Output and reporting
│       └── sorting.h           # Sorting algorithms
└── src/                        # Source files
    ├── main.cpp                # Main algorithm implementation
    ├── parameters/             # Parameter implementations
    │   ├── hardcoded.cpp       # Constant definitions
    │   ├── optimization.cpp    # Variable definitions
    │   ├── others.cpp          # Auxiliary variables and processing
    │   └── random_generators.cpp # RNG implementations
    └── sideFunctions/          # Utility implementations
        ├── reporting.cpp       # Solution output functions
        └── sorting.cpp         # QuickSort and other sorting
```

## Core Components

### 1. Parameters Module (`parameters/`)

#### `hardcoded.h/cpp`
- **Problem constants**: Number of buses, stations, passengers
- **Physical constraints**: Bus capacity, speeds, time limits
- **Algorithm parameters**: Iteration limits, destroy operations
- **Cost weights**: c1, c2, c3 for objective function

#### `optimization.h/cpp`
- **Decision variables**: 
  - `xk[i][p][j]`: Passenger `p` assigned to station `j` on bus `i`
  - `yk[i][j][k]`: Bus `i` travels from station `j` to station `k`
  - `Ak[i]`: Arrival time for bus `i`
  - `Dk[i]`: Idle/delay time for bus `i`
- **Solution tracking**: Current, proposed, and best solutions
- **Memory management**: Functions for dynamic array allocation/deallocation

#### `others.h/cpp`
- **Passenger data**: Locations, desired arrival times
- **Station data**: Mandatory and optional station coordinates
- **Travel time matrices**: Pre-computed distances between all points
- **Processing functions**: Data loading and initialization

#### `random_generators.h/cpp`
- **RNG engine**: Mersenne Twister random number generator
- **Distributions**: 
  - `rand_passenger`: Random passenger selection
  - `rand_station`: Random station selection
  - `rand_percentage`: Percentage-based decisions
  - `rand_bus_weighted`: Weighted bus selection
  - `rand_destroy_count`: Number of passengers to destroy (3-5)

### 2. Side Functions Module (`sideFunctions/`)

#### `sorting.h/cpp`
- **QuickSort**: Sorts buses by ranking values
- **Median finding**: Calculates median passenger arrival time
- **Index search**: Finds station positions in routes

#### `reporting.h/cpp`
- **Solution output**: Writes final solutions to files
- **Progress tracking**: Records cost improvements over time
- **Instance saving**: Stores problem parameters

### 3. Main Algorithm (`main.cpp`)

The main algorithm follows this structure:

#### Phase 1: Initialization
1. Load parameters and process input data
2. Sort passengers by desired arrival time
3. Assign passengers to buses sequentially
4. Construct initial routes using nearest neighbor heuristic
5. Apply 2-opt local search for route improvement

#### Phase 2: Large Neighborhood Search (LNS)
```
while (iterations - last_improvement <= max_iterations):
    1. DESTROY: Remove 3-5 randomly selected passengers
    2. REASSIGN: Assign destroyed passengers to new buses
       - Rank buses by arrival time similarity
       - Use weighted selection favoring better-ranked buses
    3. REBUILD: Reconstruct routes for affected buses
       - Select closest eligible stations
       - Build routes using greedy nearest neighbor
    4. EVALUATE: Calculate cost of new solution
    5. ACCEPT: If cost improved, update current solution
```

#### Phase 3: Final Optimization
- Apply intensive 2-opt with Simulated Annealing
- Use nearest neighbor candidate lists
- Cool down temperature gradually
- Track best solution across all runs

## Algorithm Features

### Destroy-and-Repair Heuristic
- Randomly removes passengers from current solution
- Reassigns to different buses based on arrival time compatibility
- Allows exploration of different bus-passenger assignments

### 2-Opt Local Search
- Reverses route segments to reduce travel distance
- Applied after initialization and at the end
- Uses Simulated Annealing in final phase for better exploration

### Diversification Strategies
- **Second/Third station selection**: Probabilistically chooses non-optimal stations
- **Weighted bus selection**: Biases toward better-ranked buses but allows variety
- **Random destroy count**: Varies neighborhood size (3-5 passengers)

### Feasibility Constraints
- **Capacity**: Ensures buses don't exceed maximum capacity
- **Time windows**: Respects passenger arrival time constraints
- **Walking distance**: Limits passenger-to-station walking time
- **Route connectivity**: Maintains valid station sequences

## Building and Running

### Prerequisites
- CMake 3.22 or higher
- C++17 compatible compiler (Clang, GCC, or MSVC)
- (Optional) AddressSanitizer for memory debugging

### Build Instructions

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build the project
cmake --build . --config Debug

# Run the executable
./DRFS_LNS
```

### Debug Build
The project is configured to use AddressSanitizer in Debug mode for detecting memory issues:
```bash
cmake --build . --config Debug
```

### Release Build
For optimal performance:
```bash
cmake --build . --config Release
```

## Configuration

### Input Files
Place instance files in `Data/instances/` directory with passenger locations and desired arrival times.

### Adjusting Parameters
Edit values in `src/parameters/hardcoded.cpp`:
- `nRUN`: Number of independent runs
- `f_IT`: Maximum iterations without improvement
- `ndestroy`: Number of passengers to destroy per iteration
- `c1, c2, c3`: Cost function weights
- `secondS, thirdS`: Station selection diversification thresholds

## Output

Results are written to `Data/results/`:
- `Runs_[instance].txt`: Cost and runtime for each run
- Solution files with detailed assignments and routes
- Progress tracking with improvement timestamps

## Performance

The algorithm tracks:
- **Best cost**: Lowest objective value found
- **Runtime**: Computational time to best solution
- **Last improvement time**: When the best solution was discovered
- **Progress curve**: Cost evolution over iterations

## Development

### VS Code Integration
The project includes VS Code configuration:
- **Debug**: F5 to build and debug with breakpoints
- **Build tasks**: Ctrl+Shift+B for quick compilation
- **IntelliSense**: Full code completion with compile_commands.json

### Memory Safety
- AddressSanitizer enabled in Debug builds
- Proper cleanup in `deletePointers()` function
- RAII principles for vector management

## References

This implementation is based on Large Neighborhood Search metaheuristics combined with local search techniques for vehicle routing problems with time windows and optional stops. This si the code that was used on an academic paper. 


## Author

Bryan Galarza (2025)

