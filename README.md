# Path_Planning
This “All Rivers” algorithm estimates a path from one or more training trajectories. Based on training data, the estimated path allows for path starts at any location on, or near one or more of its training trajectories. In doing so, it avoids obstacles, resolves backtracking in training, and avoids training loops in training trajectories.

# Description
This project was motivated by a “Learning by Demonstration” project for industrial robots. The goal was for a robot to observe multiple attempts by humans to complete a task. Specifically, the robot would track hand trajectories, calculate an average trajectory, and then perform that task with an end effector. Originally, the approach was to calculate an average trajectory from multiple trajectory observations. However, no algorithm exists that calculates a true average, and calculating a true average trajectory in a statistically rigorous way appears unachievable. Moreover, algorithms that do calculate something approximating an average have inherent problems. For example, if two vehicles take alternative paths around an obstacle, one to the left, the other to the right, the average trajectory will take the center path, directly into the obstacle!
All Rivers uses a machine learning approach wherein training trajectories are iteratively used to generate, and improve its model. Model history is captured in a grid of “nodes” that span task space. Thus, there is no need to store a record of training trajectories. The term "All Rivers" derives its name from King Solomon’s quote “all rivers run into the sea”. Accordingly, the estimated path assumes training trajectories all have a common destination. Inspiration for this work derives from brain physiology in the entorhinal cortex that allows animals to navigate mazes, so-called “grid cells”.
Model training does not require a collection of trajectories that are processed together. Instead, the model is updated and improved iteratively. The model is update-able even as a trajectory is being formed in real time. It will avoid obstacles (assuming they were avoided in training trajectories) by adhering to an approximate average of a valid path. Moreover, PVF automatically cuts out trajectory loops, or equivalently, cases where the trajectory crosses itself. While this is advantageous for many use-cases, it is not suitable for trajectories where loops are desired. Finally, All Rivers takes into account changing trajectory velocity. If different trajectories are captured with a constant sampling rate, higher and lower velocities will result in more or less distance respectively between succeeding coordinate points. All Rivers approximately calculates average speed across portions of the training trajectories. In terms of algorithmic complexity, training and trajectory estimation are in linear time with respect to trajectory count and coordinate count. In terms of space complexity, PVF is O(n<sup>2</sup>) with respect to the density of nodes being used to span the trajectory space, and the size of that space.
At the time of this writing, All Rivers works for 2D cases, but a 3D version is being contemplated. Notwithstanding, All Rivers efficiently solves certain problems, such as finding the correct path through a maze, or avoiding obstacles. Use cases may include, but are not limited to training by demonstration for plainer moves of a SCARA robot, path planing for AGVs, navigating a maze, or traffic flow modeling.

# Getting Started
## Dependencies
* Developed using VS Code in Ubuntu 24.04, and compiled using g++ (Ubuntu 13.3.0-6ubuntu2~24.04) 13.3.0.
* C++ code uses the standard library.
* In the example provided, All Rivers generates estimated trajectories. Phython is used to plot the outputted trajectories in a maze. If visualization of trajectories is desired, Matplotlib 3.9 was used for the Python script, called "plot_results," located in the "outputs" directory.
* An additional Python script, called "trajectory_creator.py," is provided in the "training" directory. This simple tool allows users to generate trajectories using mouse clicks. The trajectory is then saved in a compatible .txt file format for training. It requires the Turtle library.
## Installing
* Download and unzip (or clone) into your working directory. The src directory contains .cpp files. The include directory contains .h files. The training directory contains a sample of training trajectories. The output directory gets populated with estimated trajectories. It also contains an image file of the maze used by the sample training trajectories
* In order to follow the README instructions, see the main.cpp file in src.
## Executing program
* Build main.cpp. Execute main in the src directory using ./main.
### Creating and Training a New Model
* A collection of training trajectories are provided in the training directory for these directions. Alternatively, trajectory_creator.py allows for the generation of additional training trjectories. However, it is recomended that the main.cpp file be examined, and executed first for understanding. The following instructions step through the logic of main.cpp:
* Create a training object with the desired node spacing. Grid spacing defines the distance between training nodes. Finding a spacing that results in acceptable estimated pathes is best approached with trial and error. As a rule of thumb, this distance should be comparable in size, or larger than the typical distance between successive trajectory points. Once a suitable grid spacing is found, this parameter is non-adjustable without retraining. Assuming the desired node spacing is 85:
```
float grid_spacing = 85;
vector<vector<float>> traj; // Declares a variable for reading in training trajectories.
```
* Read in a trajectory given here by the name "Explore entire maze.txt". A .txt file with this name is located in the training directory. This trajectory traverses every cooridor or a maze. The titles of training trajectories in the training directory provide a clue to what heuristics were used to generate that trajectory. After reading in the trajectory, instantiate the BuildGrid object, "train". The constructor requires a trajectory to establish coordinate frame extents, as well as a grid spacing. This is enough to estimate trajectories. However, estimation will become more robust as training this object is performed with additional training trajectories:
```
traj = read_traj("Explore entire maze.txt"); // Read-in training trajectory
 BuildGrid train(traj, grid_spacing); // Generate new grid
```
* Read in the "Left Only.txt" trajctory and then train the train object. Notice that the grid spacing is no longer specified:
```
traj = read_traj("Left Only.txt");
train.trainOnTrajectory(traj); // Update new grid with another trajectory
```
* Train on a third trajectory as was done in the previous step. The main file repeates this procedure for the six remaining trajectories in the training directory. These instructions will skip that training:

```
traj = read_traj("Left Right.txt");
train.trainOnTrajectory(traj); // Update ...
```
* After training is complete, a pointer to the training model is obtained as follows. This pointer can be used to save the model, can be read in from a saved model, passed to another training object for additional training, or passed to a path finding object for estimating a trajectory.

```
BuildGrid::GridInfo* model = train.getModel();
```
* Here the model is saved to a file called "model.txt" The file will populate in the ouputs directory.:
```
save_model(model, "model.txt");
```
### Loading and Estimating Pathes from a Saved Model
* With the model saved, we may destroy the train object. Assuming that was done, the following instructions demonstrate reading the model back in as a pointer called "saved_model".
```
BuildGrid::GridInfo* saved_model = read_model("model.txt");
```
* The read model is now used to generate an estimated trajectory. All Rivers allows for arbitrary start points. However, the model being used in this example traverses a maze. Not all start points are valid. For example, a start point located within a maze wall would result in an indeterminite path to the maze exit. To simplify start point selection, the model retains the start point of the last training trajectory. All training trajectories for this example started at the maze entrance, and eventually ended at the maze exit. Therefore, the following command will provide a start point located at the maze entrance:
```
vector<float> start = saved_model->last_start_pt;
```
* Use the FindPath class to create an estimate object using the model, and a start point as arguments:
```
FindPath estimate(saved_model, start);
```
* Obtain the estimated trajectory as follows:
```
vector<vector<float>> calculated_path = estimate.get_path();
```
* The estimated trajectory is now saved to the outputs directory as follows:
```
write_traj(calculated_path, "estimated_path_from_start.txt");
```
* Several more calculated trajectories are generated from other start points, and saved. The start point below estimates a path from one of the dead ends in the maze:
```
    start = {1490, 731}; // Start from a dead end
    estimate.find_path(start);
    calculated_path = estimate.get_path();
    write_traj(calculated_path, "estimated_path_from_dead_end.txt");
```
* One more estimated path is generated from the start point of an intersection:
```
    start = {-967, 402}; // Start from an intersection
    estimate.find_path(start);
    calculated_path = estimate.get_path();
    write_traj(calculated_path, "estimated_path_from_intersection.txt");
```
* The ouptut directory should now contain three estimated trajectory files. In order to visualize them within the maze run the Python script plot_results.py, located in the outputs directory.


### Concluding Remarks
Grid spacing selection has a large effect on model performance. If the spacing is too small (dense grid) a larger training set is required. If the grid spacing is too large the calculated trajectory will tend to round off sharper trajectory curves, akin to high frequency filtering.
Because training trajectories may start and end at different locations, it is difficult to specify the calculated trajectory’s end point. In general, if training trajectories all have dissilar end points, All Rivers is biased to selecting an end point based on the trajectories that were most recently trained.
After training on a small number of trajectories, the calculated trajectories are sometimes cut short. All Rivers generates training for a finite coordinate frame space. If training trajectories span a larger space than previous ones, this will enlarge the model.
# Author
Matt Simkins
simkinsmatt@hotmail.com
# Version History
* 0.1, Initial Release
# License
GNU General Public License v3.0
