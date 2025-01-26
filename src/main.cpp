#include <iostream>
#include <vector>
#include "build_grid.h"
#include "find_path.h"
#include "file_handling.h"
#include "path_functions.h" //included for testing only

using namespace std;


int main() {
    
    float grid_spacing = 85; // May involve tuning
    vector<vector<float>> traj; // Training trajectory

    traj = read_traj("Explore entire maze.txt"); // Read-in training trajectory
    BuildGrid train(traj, grid_spacing); // Generate new grid

    traj = read_traj("Left Only.txt");
    train.trainOnTrajectory(traj); // Update new grid with another trajectory

    traj = read_traj("Left Right.txt");
    train.trainOnTrajectory(traj); // Update ...

    traj = read_traj("Left then right.txt");
    train.trainOnTrajectory(traj);

    traj = read_traj("Random 2.txt");
    train.trainOnTrajectory(traj);

    traj = read_traj("Random 3.txt");
    train.trainOnTrajectory(traj);

    traj = read_traj("Right Only.txt");
    train.trainOnTrajectory(traj);

    traj = read_traj("Random 1.txt");
    train.trainOnTrajectory(traj);

    traj = read_traj("zOptimal path fine.txt");
    train.trainOnTrajectory(traj);
    
    // Obtain pointer to trained model
    BuildGrid::GridInfo* model = train.getModel();
    
    // Save model to ouput directory
    save_model(model, "model.txt");

    // Alternatively, a trained model can be read in from output directory
    BuildGrid::GridInfo* saved_model = read_model("model.txt");
    
    // Obtain the start point from training trajectory last trained on
    vector<float> start = saved_model->last_start_pt;

    // Generate estimated path based on trained model
    FindPath estimate(saved_model, start);

    // Obtain path estimate to maze end
    vector<vector<float>> calculated_path = estimate.get_path();

    // Save estimated path from start point
    write_traj(calculated_path, "estimated_path_from_start.txt");

    // Estimate a path from dead end
    start = {1490, 731}; // Start from a dead end
    estimate.find_path(start);
    calculated_path = estimate.get_path();
    
    // Save estimated path from new start point
    write_traj(calculated_path, "estimated_path_from_dead_end.txt");
    

    // Estimate a path from intersection
    start = {-967, 402}; // Start from an intersection
    estimate.find_path(start);
    calculated_path = estimate.get_path();
    
    // Save estimated path from new start point
    write_traj(calculated_path, "estimated_path_from_intersection.txt");


    return 0;
}