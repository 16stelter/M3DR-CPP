# M3DR-CPP Solver

This is a solver for the Multi-modal Multiple Disconnected Regions Coverage Path Planning problem. The solver detects the disconnected regions based on
slope and finds a complete coverage path by first solving the TSP in each region and then connecting all regions by solving another TSP.

## Requirements

The solver expects the following software libraries to be available. Different versions of these libraries might work,
but you need to change the CMakeList and parts of the source code in that case. Some files for CGAL have been modified, the modified files are included in the resources folder. Simply add them to your CGAL installation.

* CGAL-6.0
* LKH-3.0.11

Also note, that the current scale and penalties that are set expect the LKH solver to be modified to accept long long int. If you do not want this, change all places in code that are set to a factor of 10 to a smaller factor instead. This will reduce accuracy and return costs that are slightly smaller than the actual costs. If you want to modify LKH to match this, the easiest (brute force) way to do so is going to the LKH SRC folder and executing

```find . -type f -name ".c" -or -name ".h" | xargs sed -i 's/\bint\b/long long int/g'```

Please be aware that this is not necessarily the smartest thing to do as this will replace every integer in the entire LKH library with a long long int. If you want to take the time, you can go through the code and only replace those integers that are relevant to the weights. That would result in a more performant solver. After making that replacement, you need to make LKH again.

## How to use

The package contains a ros2 action server. You can start the server using

```ros2 run coverage_path_planning CoveragePathPlanning```

You can send a request to the server using 

```ros2 action send_goal /coverage_action coverage_path_planning/action/CoverageAction "{map_filepath: 'your_map_file.stl'}"```

Additional parameters you can add to your request:

debug_mode: 0 (default) - no debug, 1 - renders path only, 2 - renders path and mesh with region colors. Rendering is blocking execution until renderer closes.
reduction_factor: 0.0 (default) - no mesh reduction, 0.0 < r < 1.0 - reduces number of triangles until the specified fraction remains.
