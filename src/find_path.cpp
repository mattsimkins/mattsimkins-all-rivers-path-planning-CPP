#include <iostream>
#include "find_path.h"
#include "build_grid.h"
#include "path_functions.h"

using namespace std;

//Constructor for new model
FindPath::FindPath(BuildGrid::GridInfo* trained_model, vector<float>& start_point)
    : model(trained_model), start_c(start_point) {
    calculate_path();
}
FindPath::~FindPath() {}

void FindPath::calculate_path(){
    vector<float> loc;
    vector<float> next_loc;
    calc_traj.push_back({start_c[0] + model->shift[0],
            start_c[1] + model->shift[1]});
    running_path_length = 0;
    bool stop_calc = false;

        // Continues growing calculated path until a stop_calc is false
        while (stop_calc == false){
            
            // Start loop from sast coordinate added
            loc = calc_traj[(calc_traj.size() - 1)];

            // Check that nodes in the triad exceed grid space limits
            stop_calc = checkExtents(loc, model->grid.size(),
                                       model->grid[0].size(), model->d);
            
            // Gather indices neighboring nodes
            vector<vector<int>> indices = find_trident(loc, model->d);

            // Check for negative indices
            for (int i = 0; i < indices.size(); i++){
                for (int j = 0; j < indices[i].size(); j++){
                    if (indices[i][j] < 0){
                        throw runtime_error("Possibly bad start point.");
                    }
                }
            }

            // Gather vectors recorded in triad of neighboring nodes
            vector<float> vec_left = {model->grid[indices[0][0]][indices[0][1]][0],
                model->grid[indices[0][0]][indices[0][1]][1]};
            
            vector<float> vec_right = {model->grid[indices[1][0]][indices[1][1]][0],
                model->grid[indices[1][0]][indices[1][1]][1]};

            vector<float> vec_center = {model->grid[indices[2][0]][indices[2][1]][0],
                    model->grid[indices[2][0]][indices[2][1]][1]};

            vector<vector<float>> triad_vecs = {{vec_left}, {vec_right}, {vec_center}};
            
            // Determine if triad nodes were visited previously
            bool left_visited = false;
            bool right_visited = false;
            bool center_visited = false;
            if (norm(vec_left, {0, 0}) != 0) { 
                left_visited = true;
            }
            
            if (norm(vec_right, {0, 0}) != 0) { 
                right_visited = true;
            } 
            
            if (norm(vec_center, {0 ,0}) != 0) {
                center_visited = true;
            }
            
            // Describes wich nodes were visited for interpolation
            vector<bool> visited_nodes = {left_visited, right_visited, center_visited};

            // Calculated pseudo average proceeds based on 3 cases
            int num_nodes_visited = left_visited + right_visited + center_visited;

            // Case 1 - All nodes are empty so abort, model fails
            if (num_nodes_visited == 0) {
                stop_calc = true;
                cout << "Model failed to find a path" << endl;

            // Case 2 - Two of three nodes are empty, but recoverable
            } else if (num_nodes_visited == 1){
                next_loc = two_empty_nodes(loc, triad_vecs, visited_nodes, indices);
                    
            // Case 3 - One of three nodes were zero
            } else if (num_nodes_visited == 2){
                next_loc = one_empty_node(loc, triad_vecs, visited_nodes, indices);
                
            // case 4 - All 3 nodes are non-zero (best case and most typical)        
            } else {
                next_loc = no_empty_node(loc, triad_vecs, indices);
            }

            // Update running path length to ensure path does not run on forever
            float new_length = norm(next_loc, loc);
            running_path_length = running_path_length + new_length;
            
            // Check if done
            if (next_loc == loc)
                break;
            
            // Check for excessive coordinate count
            if (calc_traj.size() > model->max_coord_count + 
                    round(0.01 * POINT_COUNT_MARGIN * model->max_coord_count)) {
                stop_calc = true;
            }
            
            // Check for excessively short segments
            if (model->shortest_segment > new_length + 
                round(0.01 * SHORTEST_SEGMENT_MARGIN * new_length)) {
                stop_calc = true;
            }
            
            // Check that trajectory path is not much longer than average            
            if (model->average_path_length < running_path_length) {
                stop_calc = true;
            }       
            calc_traj.push_back(next_loc); // Grow trajectory by one coordinate
        } // End while loop
} 

// Public function
vector<vector<float>> FindPath::get_path(){
    vector<vector<float>> traj_c(calc_traj.size(), vector<float>(2));
    cout << "calc_traj size is: " << calc_traj.size() << endl;
    for (int i = 0; i < calc_traj.size(); i++){
        traj_c[i][0] = calc_traj[i][0] - model->shift[0];
        traj_c[i][1] = calc_traj[i][1] - model->shift[1];
    }
    return traj_c;
}

void FindPath::update_empty_node(const vector<float>& vec, const vector<int>& index){
    if ((this->model->grid[index[0]][index[1]][0] != 0) &&
        (this->model->grid[index[0]][index[1]][1] != 0)){
        throw runtime_error("Trying to overwrite non-zero grid element.");
    }

    this->model->grid[index[0]][index[1]][0] = vec[0];
    this->model->grid[index[0]][index[1]][1] = vec[1];
}

vector<float> FindPath::two_empty_nodes(
                                        const vector<float>& loc,
                                        const vector<vector<float>>& triad_vecs,
                                        const vector<bool>& visited_nodes, 
                                        const vector<vector<int>>& indices) {
    
    vector<float> next_loc; // Next location to add to trajectory
    
    // Calculate location of nodes
    vector<float> loc_left = coord_from_ind(indices[0], model->d);
    vector<float> loc_right = coord_from_ind(indices[1], model->d);
    vector<float> loc_center = coord_from_ind(indices[2], model->d);
    
    // Right and center nodes were empty, use left to update
    if (visited_nodes[0] == true &&
        visited_nodes[1] == false &&
        visited_nodes[2] == false) {
        vector<float> loc_left_points2 = add_vectors(loc_left, triad_vecs[0]);
        vector<float> vec_right = subtract_vectors(loc_left_points2, loc_right);
        vector<float> vec_center = subtract_vectors(loc_left_points2, loc_center);
        
        // Populate any empty nodes
        update_empty_node(vec_right, indices[1]);
        update_empty_node(vec_center, indices[2]);

    // #Left and center nodes were empty, use right to update
    } else if (visited_nodes[0] == false &&
               visited_nodes[1] == true &&
               visited_nodes[2] == false) {
        vector<float> loc_right_points2 = add_vectors(loc_right, triad_vecs[1]);
        vector<float> vec_left = subtract_vectors(loc_right_points2, loc_left);
        vector<float> vec_center = subtract_vectors(loc_right_points2, loc_center);
        
        // Populate any empty nodes
        update_empty_node(vec_left, indices[0]);
        update_empty_node(vec_center, indices[2]);

    // Left and right nodes were empty, use center to update
    } else if (visited_nodes[0] == false &&
               visited_nodes[1] == false &&
               visited_nodes[2] == true) {
        vector<float> loc_center_points2 = add_vectors(loc_center, triad_vecs[2]);
        vector<float> vec_left = subtract_vectors(loc_center_points2, loc_left);
        vector<float> vec_right = subtract_vectors(loc_center_points2, loc_right);
        
        // Populate any empty nodes
        update_empty_node(vec_left, indices[0]);
        update_empty_node(vec_right, indices[1]);
    }
    // All vectors point to same place, using left (triad_vecs[0]) is arbitrary
    next_loc = add_vectors(loc, triad_vecs[0]);
    return next_loc;
}

vector<float> FindPath::one_empty_node(
                                       const vector<float>& loc,
                                       const vector<vector<float>>& triad_vecs,
                                       const vector<bool>& visited_nodes, 
                                       const vector<vector<int>>& indices) {
    // Find distances to neighboring grid nodes
    float dist2left = dist2node(loc, indices[0], model->d);
    float dist2right = dist2node(loc,indices[1], model->d);
    float dist2center = dist2node(loc, indices[2], model->d);
    
    vector<float> next_loc; // Next location to add to trajectory

    // Left node empty, use right and center to calculate next location
    if (visited_nodes[0] == false &&
        visited_nodes[1] == true && 
        visited_nodes[2] == true) {
                                
        // Determine weights based on distances to nodes
        float den = dist2right + dist2center;
        float weight_right = dist2right/den;
        float weight_center = dist2center/den;
        
        // Weight the vectors
        vector<float> vec_right_weighted = scalar_multiply(triad_vecs[1], weight_right);
        vector<float> vec_center_weighted = scalar_multiply(triad_vecs[2], weight_center);
        
        // Calculate sum of weighted vectors
        vector<float> vec_r_plus_c = add_vectors(vec_right_weighted, vec_center_weighted);
        next_loc = add_vectors(loc, vec_r_plus_c);
        
        // Populate empty node
        vector<float> loc_left = coord_from_ind(indices[0], model->d);
        vector<float> vec_left = subtract_vectors(next_loc, loc_left);
        update_empty_node(vec_left, indices[0]);
    
    // Right node empty, use center and left
    } else if (visited_nodes[0] == true &&
               visited_nodes[1] == false &&
               visited_nodes[2] == true) {
        // Determine weights based on distances to nodes
        float den = dist2left + dist2center;
        float weight_left = dist2left/den;
        float weight_center = dist2center/den;
        
        // Weight the vectors
        vector<float> vec_left_weighted = scalar_multiply(triad_vecs[0], weight_left);
        vector<float> vec_center_weighted = scalar_multiply(triad_vecs[2], weight_center);
        
        // Calculate sum of weighted vectors
        vector<float> vec_l_plus_c = add_vectors(vec_left_weighted, vec_center_weighted);
        next_loc = add_vectors(loc, vec_l_plus_c);
        
        // Populate empty node
        vector<float> loc_right = coord_from_ind(indices[1], model->d);
        vector<float> vec_right = subtract_vectors(next_loc, loc_right);
        update_empty_node(vec_right, indices[1]);

    // Center node empty, use left and right    
    } else {
        // Determine weights based on distances to nodes
        float den = dist2left + dist2right;
        float weight_left = dist2left/den;
        float weight_right = dist2right/den;
        
        // Weight the vectors
        vector<float> vec_left_weighted = scalar_multiply(triad_vecs[0], weight_left);
        vector<float> vec_right_weighted = scalar_multiply(triad_vecs[1], weight_right);
        
        // Calculate sum of weighted vectors
        vector<float> vec_l_plus_r = add_vectors(vec_left_weighted, vec_right_weighted);
        next_loc = add_vectors(loc, vec_l_plus_r);
        
        // Populate empty node
        vector<float> loc_center = coord_from_ind(indices[2], model->d);
        vector<float> vec_center = subtract_vectors(next_loc, loc_center);
        update_empty_node(vec_center, indices[2]);
    }
    return next_loc;
}

vector<float> FindPath::no_empty_node(
                                      const vector<float>&loc,
                                      const vector<vector<float>>& triad_vecs,
                                      const vector<vector<int>>& indices) {

    vector<float> next_loc; // Next location to add to trajectory
    
    // determine distances to neighboring grid nodes
    float dist2left = dist2node(loc, indices[0], model->d);
    float dist2right = dist2node(loc, indices[1], model->d);
    float dist2center = dist2node(loc, indices[2], model->d);
    
    // Determine weights based on distances to nodes
    float den = dist2left + dist2center + dist2right;
    float weight_left = (dist2center + dist2right - dist2left)/den;
    float weight_right = (dist2center + dist2left - dist2right)/den;
    float weight_center = (dist2left + dist2right - dist2center)/den;
    
    // Weight the vectors
    vector<float> vec_left_weighted = scalar_multiply(triad_vecs[0], weight_left);
    vector<float> vec_right_weighted = scalar_multiply(triad_vecs[1], weight_right);
    vector<float> vec_center_weighted = scalar_multiply(triad_vecs[2], weight_center);
    
    // Sum weighted vectors
    vector<float> vec_l_plus_r = add_vectors(vec_left_weighted, vec_right_weighted);
    vector<float> vec_sum = add_vectors(vec_l_plus_r, vec_center_weighted);
    next_loc = add_vectors(loc, vec_sum);
    return next_loc;   
}