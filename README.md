# M3DR-CPP Solver

This is a solver for the Multi-modal Multiple Disconnected Regions Coverage Path Planning problem. The solver detects the disconnected regions based on
slope and finds a complete coverage path by first solving the TSP in each region and then connecting all regions by solving another TSP.

## Requirements

The solver expects the following software libraries to be available. Different versions of these libraries might work,
but you need to change the CMakeList and parts of the source code in that case. Some files for CGAL have been modified, the modified files are included in the resources folder. Simply add them to your CGAL installation.

* CGAL-6.0
* LKH-3.0.11

Also note, that the current scale and penalties that are set expect the LKH solver to be modified to accept long long int. If you do not want this, change all places in code that are set to a factor of 10 to a smaller factor instead. This will reduce accuracy and return costs that are slightly smaller than the actual costs. If you want to modify LKH to match this, the easiest (brute force) way to do so is going to the LKH SRC folder and executing

```find . -type f \( -name "*.c" -o -name "*.h" \) | xargs sed -E -i 's/\bint\b(?! compare)/long long int/g'```

```find . -type f \( -name "*.c" -o -name "*.h" \) -print0 | xargs -0 sed -i 's/%d/%lld/g'```

Please be aware that this is not necessarily the smartest thing to do as this will replace every integer in the entire LKH library with a long long int. If you want to take the time, you can go through the code and only replace those integers that are relevant to the weights. That would result in a more performant solver. After making that replacement, you need to make LKH again. Depending on your compiler, 

## How to use

The package contains a ros2 action server. You can start the server using

```ros2 run coverage_path_planning CoveragePathPlanning```

You can send a request to the server using 

```ros2 action send_goal /coverage_action coverage_path_planning/action/CoverageAction "{map_filepath: 'your_map_file.stl', runs: 3}"```

Number of runs influences how often LKH will recalculate the solution with different starting conditions. Higher number may slightly increase solution quality, but significantly increases calculation time.

Additional parameters you can add to your request:

* debug_mode: 0 (default) - no debug, 1 - renders path only, 2 - renders path and mesh with region colors. Rendering is blocking execution until renderer closes.
* reduction_factor: 0.0 (default) - no mesh reduction, 0.0 < r < 1.0 - reduces number of triangles until the specified fraction remains.
* use_global_tsp: Overrides the proposed approach to instead consider the task as a single global TSP problem. Significantly increases calculation time.

## Integration with Gazebo

This package provides demo scripts to execute a coverage mission in gazebo. This has been tested on Ros2 Jazzy with Gazebo Harmomic. To run these demos, you require the following additional packages:

* ros2-gz-sandbox: git@github.com:16stelter/ros2-gz-sandbox.git
* simple_terrain_gen: git@github.com:16stelter/simple_terrain_gen.git

To start the simulation, execute the following steps:

* ```ros2 run map_gen MapGen```
* ```ros2 run coverage_path_planning CoveragePathPlanning```
* ```ros2 run coverage_path_planning purepursuit.py --ros-args -r __ns:=/multimodal -p cmd_vel_topic:=/cmd_vel_ground -p action_topic:=/follow_path_ground```
* ```ros2 run coverage_path_planning flytogoal.py --ros-args -r __ns:=/multimodal -p cmd_vel_topic:=/cmd_vel_air```
* ```ros2 run coverage_path_planning multimodal_coverage.py```
* Copy the generated output.stl into the worlds folder of ros2-gz-sandbox
* ```ros2 launch rl ros2_gz_sandbox sim_single.launch.py type:=multimodal robot_ns:=multimodal world_name:=output```
* Adjust the position of the robot as indicated in your terminal. Make sure the robot is located close above the ground.
* Unpause the simulation and confirm in the terminal.

In a future version, this will be provided as a launch script. 
