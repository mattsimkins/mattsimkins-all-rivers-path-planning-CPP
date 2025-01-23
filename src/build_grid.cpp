#include <iostream>
#include <cmath> 
#include <limits>
#include <cmath>
#include "build_grid.h"
#include "path_functions.h"

using namespace std;


BuildGrid::BuildGrid(vector<vector<float>>& trajectory, float& node_spacing)
                : traj_c(trajectory) {
        model.d = node_spacing;
    model.grid_update_count = 0;
    model.average_path_length = 0;
    model.max_coord_count = 0;
    model.shortest_segment = model.d;
    model.last_start_pt = trajectory[0];
    cSpaceExtents = calculateExtents_c();
    model.shift = {-cSpaceExtents[0], -cSpaceExtents[2]};
    initializeGrid();
    trainGrid(traj_c);
}

BuildGrid::~BuildGrid() {}

// Public functions
BuildGrid::GridInfo* BuildGrid::getModel() {
    return &model;
}

void BuildGrid::trainOnTrajectory(vector<vector<float>>& traj){
    trainGrid(traj);
}

vector<float> BuildGrid::getStartPoint() {
    return model.last_start_pt;
}


void BuildGrid::fixDuplicates(){
    // Check for duplicate points in the trajectory
    for (int i = 0; i < traj_c.size(); i++) {
        for (int j = i + 1; j < traj_c.size(); j++) {
            if (traj_c[i][0] == traj_c[j][0] && traj_c[i][1] == traj_c[j][1]) {
                traj_c.erase(traj_c.begin() + j);
            }
        }
    }
}

// Increment grid size if trajectory exceeds current extents by increments of d
void BuildGrid::checkExtents_c(){
    vector<float> trajExtents_c = calculateExtents_c();
    
    // Check if current training trajectory fits within S-space
    vector<int> expandIncrement; // Amount S-space is possibly enlarged
    float sign = -1;
    for (int i = 0; i < trajExtents_c.size(); ++i) {
        float diff = trajExtents_c[i] - cSpaceExtents[i];
        if ((diff*sign > model.d) && (i%2 == 0)) {
            expandIncrement.push_back(ceil(abs(diff)/model.d));
        } else if ((diff*sign > model.d) && (i%2 != 0)){
            expandIncrement.push_back(ceil(abs(diff)/(model.d*Y_TRI)));
        } else{
            expandIncrement.push_back(0);
        }
        sign = sign * -1;
    }
    if (expandIncrement[0] > 0 || expandIncrement[1] > 0 ||
                            expandIncrement[2] > 0 || expandIncrement[3] > 0) {
        cSpaceExtents[0] -= expandIncrement[0]*model.d;
        cSpaceExtents[1] += expandIncrement[1]*model.d;
        cSpaceExtents[2] -= expandIncrement[2]*model.d*Y_TRI;
        cSpaceExtents[3] += expandIncrement[3]*model.d*Y_TRI;
        
        // Save old grid and initialize new, resized grid
        vector<vector<vector<float>>> old_grid = model.grid;

        // Calculate C-Space to S-Space transformation vector
        model.shift = {-cSpaceExtents[0], -cSpaceExtents[2]};

        // Generate an grid large enough to span S-space
        initializeGrid();

        // Enlarging S-space with larger grid
        for (int i = 0; i < old_grid.size(); i++) {
            for (int j = 0; j < old_grid[0].size(); j++) {
                for (int k = 0; k < old_grid[0][0].size(); k++) {
                    model.grid[i + expandIncrement[0]][j 
                    + expandIncrement[2]][k] = old_grid[i][j][k];
                }
            }
        }
    }

    // Obtain new shift vector for enlarged space
    model.shift = {-cSpaceExtents[0], -cSpaceExtents[2]};
}

vector<float> BuildGrid::calculateExtents_c(){
 
    // Find x extents
    float minX = numeric_limits<float>::max();
    float maxX = numeric_limits<float>::lowest();
    for (const auto& row : traj_c) {
        minX = min(minX, row[0]);
        maxX = max(maxX, row[0]);
    }

    
    // Find y extents
    float minY = numeric_limits<float>::max();
    float maxY = numeric_limits<float>::lowest();
    for (const auto& row : traj_c) {
        minY = min(minY, row[1]);
        maxY = max(maxY, row[1]);
    }
     
    // Add padding to the extents
    vector<float> trajExtents = {
                                minX - model.d,
                                maxX + model.d,
                                minY - model.d,
                                maxY + model.d
                                };
    return trajExtents;
}

//  Initialize grid sized for trajectory
vector<vector<vector<float>>> BuildGrid::sizeGrid(
                                                  const int& rows,
                                                  const int& cols,
                                                  const int& depth) {
    vector<vector<vector<float>>> resized_grid(
                                            rows, vector<vector<float>>(
                                            cols, vector<float>(
                                            depth, 0.0)));
    return resized_grid;
    }

// Initialize grid with a trajectory for size, and node spacing
void BuildGrid::initializeGrid() {
    gridSizeX = cSpaceExtents[1] - cSpaceExtents[0];
    gridSizeY = cSpaceExtents[3] - cSpaceExtents[2];
    int count_i = ceil(gridSizeX/model.d); //Along x in C-Space
    int count_j = ceil(gridSizeY/(model.d*Y_TRI)); //Along y in C-Space
    model.grid = sizeGrid(count_i, count_j, 2);
}

// Find the three closest nodes to the given trajectory coordinate
void BuildGrid::trainGrid(const vector<vector<float>>& traj_c) {
    fixDuplicates(); // Removes duplicate, consecutive trajectory points
    checkExtents_c(); // Check if traj_c requires a larger grid
    
    // Calculate metrics for the trajectory
    metrics traj_metrics = trajectory_metrics(traj_c);

    // Update average path length with running average
    model.grid_update_count++;
    model.average_path_length = (model.average_path_length
                *model.grid_update_count-1 + traj_metrics.path_length)/
                model.grid_update_count;
    
    // Track shortest length trajectory segement encountered during training
    if (model.shortest_segment > traj_metrics.shortest_segment) {
        model.shortest_segment = traj_metrics.shortest_segment;
    }

    // Update max coordinate count if necessary
    if (traj_metrics.coord_count > model.max_coord_count) {
        model.max_coord_count = traj_metrics.coord_count;
    }
 
    traj_s = traj_c; // Initialize S-Space trajectory
    for (auto& row : traj_s) {
        row[0] += model.shift[0];
        row[1] += model.shift[1];
    }

    // Loop through trajectory to update grid
    int next_ind = 0;
    for (const auto& loc : traj_s) {
        
        // Check that all nodes for location are within grid
        bool exceeded = outsideExtents(loc, model.grid.size(),
                                       model.grid[0].size(), model.d);
        if (exceeded) { 
            continue;
        }
        
        // Reached end of traj list
        if (traj_s.size() == next_ind + 1) {
            break;
        }
        
        // Vector to next location
        next_ind++;
        vector<float> loc_next = traj_s[next_ind];

        // Vector from current location to next along trajectory
        vector<float> vec2next_traj_pt = subtract_vectors(loc_next, loc);
                
        // Length of current vector
        float vec_len_vec2next_traj_pt = norm(vec2next_traj_pt, {0, 0});
        
        // Next location is close, no nodes between to update
        if (vec_len_vec2next_traj_pt < model.d) {
            vector<vector<int>> trident = find_trident(loc, model.d);
            
            // Find location of nodes
            vector<float> loc_left = coord_from_ind(trident[0], model.d);
            vector<float> loc_right = coord_from_ind(trident[1], model.d);
            vector<float> loc_center = coord_from_ind(trident[2], model.d);
            
            // Calculate vector from nodes to trajectory
            vector<float> vec_left2traj_next = subtract_vectors(loc_next,
                                                                loc_left);
            vector<float> vec_right2traj_next = subtract_vectors(loc_next,
                                                                 loc_right);
            vector<float> vec_center2traj_next = subtract_vectors(loc_next,
                                                                  loc_center);
            
            // Upate nodes with trajectory
            updateNode(vec_left2traj_next, trident[0]);
            updateNode(vec_right2traj_next, trident[1]);
            updateNode(vec_center2traj_next, trident[2]);
        
        // If next trajectory point is distant update nodes in between
        } else {

                // Find direction of increment
                vector<float> vec_direction = 
                        scalar_multiply(vec2next_traj_pt,
                        1/vec_len_vec2next_traj_pt);
                    
                // Divide current vector into peices to increment
                int n_inc = floor(vec_len_vec2next_traj_pt/model.d);

                // Step along long distance, populate nodes along the way
                for (int j = 0; j < n_inc; j++) {
                    float len_of_vec_increment = (float)j * model.d;
                    vector<float> inc_loc = add_vectors(loc,
                            scalar_multiply(vec_direction,
                            len_of_vec_increment));
                    
                    vector<vector<int>> trident = find_trident(inc_loc,
                                                               model.d);
                    
                    // Obtain coordinates of nodes
                    vector<float> loc_left = coord_from_ind(trident[0],
                                                            model.d);
                    vector<float> loc_right = coord_from_ind(trident[1],
                                                             model.d);
                    vector<float> loc_center = coord_from_ind(trident[2],
                                                              model.d);
                    
                    // Calculate vectors from nodes to next point on trajectory
                    vector<float> vec_left2traj_next = 
                                  subtract_vectors(loc_next, loc_left);
                    vector<float> vec_right2traj_next = 
                                  subtract_vectors(loc_next, loc_right);
                    vector<float> vec_center2traj_next =
                                  subtract_vectors(loc_next, loc_center);
                    
                    // Update model
                    updateNode(vec_left2traj_next, trident[0]);
                    updateNode(vec_right2traj_next, trident[1]);
                    updateNode(vec_center2traj_next, trident[2]);
            }
        }  
    }
}


void BuildGrid::updateNode(const vector<float>& vec,
                           const vector<int>& indices) {
    
    int i = indices[0];
    int j = indices[1];

    // A node's fist visit, populate nodes if empty
    if ((model.grid[i][j][0] == 0) && (model.grid[i][j][1] == 0)) {
        model.grid[i][j][0] = vec[0];
        model.grid[i][j][1] = vec[1];
    
    // For nodes not empty, update must consider previous value
    } else {
        
        // Convert vector in grid to np vector
        vector<float> hist_vec = {model.grid[i][j][0], model.grid[i][j][1]};
        
        // Dot current vector into vector saved in node
        float dot_prod = vec[0] * hist_vec[0] + vec[1] * hist_vec[1];
        
        // Length update vector
        float len_vec = norm(vec, {0, 0});
        
        // Length vector already in node
        float len_hist_vec = norm(hist_vec, {0, 0});
        
        // Find the angle between both vectors
        float cos_theta = dot_prod / (len_vec * len_hist_vec);
        float angle_rad = acos(cos_theta); // Angle in radians
        
        // Direction has changed, overwrite node
        if (angle_rad > ANGLE_THRESHOLD) {
            model.grid[i][j][0] = vec[0];
            model.grid[i][j][1] = vec[1];
        
        // New vec is same direction as node, average the vectors
        } else {
            // Biases grid towards retaining longer vectors
            float scale_fact = len_vec/(len_vec + len_hist_vec);
            vector<float> diff_vec = subtract_vectors(vec, hist_vec);
            vector<float> scaled_diff_vec = {scale_fact * diff_vec[0], 
                                             scale_fact * diff_vec[1]};
            vector<float> updated_vec = add_vectors(hist_vec, scaled_diff_vec);
            model.grid[i][j][0] = updated_vec[0];
            model.grid[i][j][1] = updated_vec[1];
        }
    }
}