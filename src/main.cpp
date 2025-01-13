#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include "build_grid.h"
#include "find_path.h"
#include "file_handling.h"
#include "path_functions.h" //included for testing only

using namespace std;


int main() {
    
    vector<vector<float>> traj = read_traj("example.txt");
    vector<vector<float>> traj1 = read_traj("example1.txt");
    
    float grid_spacing = 10;
    BuildGrid train(traj, grid_spacing);
    cout << "traj 1 done" << endl;
    train.trainOnTrajectory(traj1);
    cout << "traj 2 done" << endl;
    BuildGrid::GridInfo* model = train.getModel();

    cout << model->shift[0] << " " << model->shift[1] << endl;
    save_model(model);

    vector<float> start = {640, -320};
    FindPath estimate(model, start);

    vector<vector<float>> calculated_path = estimate.get_path();

    // for (int i = 0; i < calculated_path.size(); i++){
    //     cout << "First path x: " << calculated_path[i][0] << " y: " << calculated_path[i][1] << endl;
    // }

    BuildGrid::GridInfo* saved_model = read_model();
    //cout << model->shift[0] << " " << model->shift[1];
    start = {640, -320};
    FindPath new_estimate(saved_model, start);

    calculated_path = estimate.get_path();

    for (int i = 0; i < calculated_path.size(); i++){
        cout << "Second path x: " << calculated_path[i][0] << " y: " << calculated_path[i][1] << endl;
    }


    return 0;
}