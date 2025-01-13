#ifndef FIND_PATH_H
#define FIND_PATH_H

#include <vector>
#include "build_grid.h"
#include "file_handling.h"

using namespace std;

class FindPath {
public:
    FindPath(ModelFile* trained_model, vector<float>& start_point);

    FindPath(BuildGrid::GridInfo* trained_model, vector<float>& start_point);
    vector<vector<float>> get_path();

    ~FindPath();

private:
    void calculate_path();

    vector<float> two_empty_nodes(const vector<float>& loc,
                                  const vector<vector<float>>& triad_vecs,
                                  const vector<bool>& visited_nodes,
                                  const vector<vector<int>>& indices);

    vector<float> one_empty_node(const vector<float>& loc,
                                 const vector<vector<float>>& triad_vecs,
                                 const vector<bool>& visited_nodes,
                                 const vector<vector<int>>& indices);

    vector<float> no_empty_node(const vector<float>& loc,
                                const vector<vector<float>>& triad_vecs,
                                const vector<vector<int>>& indices);
    void update_empty_node(const vector<float>& vec, const vector<int>& index); // Updates only empty nodes
    vector<float> start_c;
    BuildGrid::GridInfo* model;
    vector<vector<float>> calc_traj;
    float running_path_length;
};

#endif // FIND_PATH_H