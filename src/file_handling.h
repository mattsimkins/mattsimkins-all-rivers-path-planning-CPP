#ifndef FILE_HANDLING 
#define FILE_HANDLING
#include <string>
#include <vector>
#include <fstream>
#include "build_grid.h"

using namespace std;

// Reads in a .txt file with a separated lisit of x and y trajectory points
vector<vector<float>> read_traj(string file_name);

// Saves model returned by BuildGrid
void save_model(BuildGrid::GridInfo* trained_model);

// Captures essential training model information
struct ModelFile {
    float d; // Node spacing
    float average_path_length; // Average length of training paths
    int grid_update_count; // Number of times grid has been updated
    int max_coord_count; // Max coordinate count in training trajectories
    float shortest_segment;
    vector<float> shift; //Shifts C-Space to S-Space
    vector<vector<vector<float>>> grid; // 3D matrix to store grid values
};

BuildGrid::GridInfo* read_model();

#endif