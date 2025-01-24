#ifndef FIND_PATH_H
#define FIND_PATH_H

#include <vector>
#include "build_grid.h"
#include "file_handling.h"

using namespace std;

class FindPath {
public:
    FindPath(BuildGrid::GridInfo* trained_model, vector<float>& start_point);
    
    vector<vector<float>> get_path();
    
    void find_path(vector<float>& start_point);
    
    ~FindPath();

private:
    void calculate_path(); // Calculates path based on trained model

    // For calculating path when training includes only one node
    vector<float> two_empty_nodes(const vector<float>& loc,
                                  const vector<vector<float>>& triad_vecs,
                                  const vector<bool>& visited_nodes,
                                  const vector<vector<int>>& indices);

    // For calculating path when training includes only two nodes
    vector<float> one_empty_node(const vector<float>& loc,
                                 const vector<vector<float>>& triad_vecs,
                                 const vector<bool>& visited_nodes,
                                 const vector<vector<int>>& indices);

    // For calcuating path where all tree nodes have values (best case)
    vector<float> no_empty_node(const vector<float>& loc,
                                const vector<vector<float>>& triad_vecs,
                                const vector<vector<int>>& indices);
    
    // Fills in empty nodes not visited during training
    void update_empty_node(const vector<float>& vec, const vector<int>& index);

    vector<float> start_c; // Starting point for estimated trajectory
    BuildGrid::GridInfo* model; // Points to model structure
    vector<vector<float>> calc_traj; // The desired estimated trajectory
    float running_length; // Monitoring for excessive length of calc_traj
};
#endif // FIND_PATH_H