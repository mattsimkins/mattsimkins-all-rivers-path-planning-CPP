#include <iostream>
//#include <cmath> 
#include <limits>
//#include <cmath>
#include "build_grid.h"
//#include "path_functions.h"
#include "find_path.h"

using namespace std;

BuildGrid::BuildGrid() {}

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


vector<float> BuildGrid::getStartPoint() {
    return model.last_start_pt;
}

// Member functions
void BuildGrid::trainOnTrajectory(vector<vector<float>>& traj){
    trainGrid(traj);

    //Estimate trajectory to refine model
    FindPath estimate(getModel(), model.last_start_pt);
    //trainGrid(traj);
    estimate.find_path(model.last_start_pt);
    trainGrid(traj);
    
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

vector<int> BuildGrid::find_closest_index(vector<float> coord, float d){
    int j_closest = round(coord[1] / (d * Y_TRI));
    int i_closest = round(coord[0] / d - 0.5);
    if (j_closest % 2 == 0) {
        int i_closest = round(coord[0] / d);
    }
    return {i_closest, j_closest};
}

vector<float> BuildGrid::index2coord(vector<int> index, float d){
    vector<float> coord = {(float)index[0] * d, (float)index[1] * d * Y_TRI};
    if (index[1] % 2 != 0) {
        coord[0] += d / 2;
    }
    return coord;
}

vector<float> BuildGrid::coord_from_ind(vector<int> ind, float d){
    float x;
    if (ind[1] % 2 == 0) {
            x = ind[0]*d;
        } else { 
            x = ind[0]*d + d/2;
        }
    
    float y = ind[1]*d*Y_TRI;
    return {x, y};
}


// Returns 3 rows of indices, to the left, right, and center of coordinate
vector<vector<int>> BuildGrid::find_trident(vector<float> coord, float d) {

    vector<int> ci = find_closest_index(coord, d); // Closest index (ci)

    // 6 directions for hexagonal grid of equatorial triangles
    const vector<vector<float>> unit_vecs = {{-1, 0}, 
                                             {-0.5, Y_TRI},
                                             {0.5, Y_TRI},
                                             {1, 0},
                                             {0.5, -Y_TRI},
                                             {-0.5, -Y_TRI}
                                             };

    // Determine if closest node is on a shifted row in x direction
    bool closest_shifted = true;
    if (ci[1] % 2 == 0){
        closest_shifted = false;
    }

    // Find the coordinate of closest node (ccn) given coordinate on trajectory
    vector<float> ccn = index2coord(ci, d);

    // Evaluate distances by dotting coorinate into unit vectors
    float dot1 = 0, dot2 = 0; // Largest, and second largest dot product
    int ind1 = 0, ind2 = 0; // Index of largest, and second largest dot product
    
    // Find the two largest dot products
    for (int i = 0; i < unit_vecs.size(); i++) {
        float dot_check = (coord[0] - ccn[0])*unit_vecs[i][0] +
                    (coord[1] - ccn[1])*unit_vecs[i][1];
        if (dot_check > dot2){
            dot2 = dot_check;
            ind2 = i;
        }
        if (dot2 > dot1){
            float dot_largest = dot2;
            int unit_vec_largest = ind2;
            dot2 = dot1;
            ind2 = ind1;
            dot1 = dot_largest;
            ind1 = unit_vec_largest;
        }
    }
    /*
    The rest of the funtion is based on spacial reasoning. In words, the
    closest node is encircled by 6 nodes that collectively form a hexagon.
    The trajectory point falls within one of the 6 sectors defined by the
    hexagon. The two largest dot products are used to determine the sector
    in which the trajectory point falls. The sector is defined by the two
    unit vectors that form the sector's boundaries. The closest node is
    the center of the hexagon, and the two largest dot products are used
    to determine the sector in which the trajectory point falls.
    */
    
    // Arbitrarily picking sextant I, all are equally valid
    if (ind1 == 0 && ind2 == 0){
        ind2 = 1;
    }

    int i_left = 0, j_left = 0, i_right = 0, j_right = 0, i_center = 0, j_center = 0;

    // Define sextant I based on dot product
    if ((ind1 == 0 && ind2 == 1) || (ind1 == 1 && ind2 == 0)){
        i_left = ci[0] - 1;
        j_left = ci[1];
        i_right = ci[0];
        if (closest_shifted) {
            i_center = ci[0];
        } else { 
            i_center = ci[0] - 1;
        }
        j_center = ci[1] + 1;

    // Define sextant II based on dot product
    } else if ((ind1 == 1 && ind2 == 2) || (ind1 == 2 && ind2 == 1)){
        if (closest_shifted) {
            i_left = ci[0];
        } else {
            i_left = ci[0] - 1;
        }
        j_left = ci[1] + 1;
        i_right = i_left + 1;
        i_center = ci[0];
        j_center = ci[1];
    

    // Define sextant III based on dot product
    } else if ((ind1 == 2 && ind2 == 3) || (ind1 == 3 && ind2 == 2)){
        i_left = ci[0];
        j_left = ci[1];
        i_right = ci[0] + 1;
        if (closest_shifted){
            i_center = ci[0] + 1;
        } else {
            i_center = ci[0];
        }
        j_center = ci[1] + 1;

        
    // Define sextant IV based on dot product
    } else if ((ind1 == 3 && ind2 == 4) || (ind1 == 4 && ind2 == 3)){
        i_left = ci[0];
        j_left = ci[1];
        i_right = ci[0] + 1;
        if (closest_shifted){
            i_center = ci[0] + 1;
        } else{
            i_center = ci[0];
        }
        j_center = ci[1] - 1;

    // Define sextant V based on dot product    
    } else if ((ind1 == 4 && ind2 == 5) || (ind1 == 5 && ind2 == 4)){
        if (closest_shifted){
            i_left = ci[0];
        } else {
            i_left = ci[0] - 1;
        }
        j_left = ci[1] - 1;
        i_right = i_left + 1;
        i_center = ci[0];
        j_center = ci[1];
        
    // Define sextant VI based on dot product    
    } else if ((ind1 == 5 && ind2 == 0) || (ind1 == 0 && ind2 == 5)){
        i_left = ci[0] - 1;
        j_left = ci[1];
        i_right = ci[0];
        if (closest_shifted){
            i_center = ci[0];
        } else {
            i_center = ci[0] - 1;
        }
        j_center = ci[1] - 1;
    }

    vector<vector<int>> triad = {
                                {i_left, j_left},
                                {i_right, j_left},
                                {i_center, j_center}
                                };
    return triad;
}

// Adds two vectors element-wise
vector<float> BuildGrid::add_vectors(vector<float> v1, vector<float> v2) {
    if (v1.size() != v2.size()) {
        throw runtime_error("Vectors must be the same size.");
    }

    vector<float> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = v1[i] + v2[i];
    }

    return result;
}


// Subtracts two vectors element-wise
vector<float> BuildGrid::subtract_vectors(vector<float> v1,
                               vector<float> v2) {
    if (v1.size() != v2.size()) {
        throw runtime_error("Vectors must be the same size.");
    }

    vector<float> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = v1[i] - v2[i];
    }

    return result;
}

// Calculates the Euclidean distance between two vectors
float BuildGrid::norm(vector<float> v1, vector<float> v2) {
    float sum = 0;
    for (int i = 0; i < v1.size(); i++) {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }
    return sqrt(sum);
}

vector<float> BuildGrid::scalar_multiply(vector<float> v, float scalar){
    vector<float> result(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * scalar;
    }
    return result;
}



float BuildGrid::dist2node(vector<float> loc, vector<int> indices, float d){
    vector<float> node_coord = index2coord(indices, d);
    return norm(loc, node_coord);
}

BuildGrid::metrics BuildGrid::trajectory_metrics(vector<vector<float>> traj){
    metrics metrics;
    float length = 0;
    for (int i = 0; i < traj.size() - 1; i++) {
        length += norm(traj[i], traj[i + 1]);
    }

    metrics.path_length = length;
    float shortest_segement = std::numeric_limits<float>::max();

    for (int i = 0; i < traj.size() - 1; i++) {
        float segment = norm(traj[i], traj[i + 1]);

        if (segment < shortest_segement) {
            shortest_segement = segment;
        }
    }

    metrics.shortest_segment = shortest_segement;
    int coord_count = traj.size();
    metrics.coord_count = coord_count;

    return metrics;
}


// Determines if a coordinate is outside the grid
bool BuildGrid::outsideExtents(vector<float> coord, int x_extent,
                    int y_extent, float d){

    bool exceeded = false;

    // The triangle of 3 nearest nodes is defined here as a "trident"
    vector<vector<int>> trident = find_trident(coord, d);

    // Coordinates on the coordinate frame axes are prohibited
    // Left is trident[0], right is trident[1], center is trident[2]
    if (coord[0] == 0 or coord[1] == 0){
        exceeded = true;
    } else if ((trident[0][0] < 0) || 
               (trident[0][1] + 1 > y_extent) || 
               (trident[0][1] < 0) || 
               (trident[1][0] + 1 > x_extent) ||
               (trident[2][1] < 0) || 
               (trident[2][1] + 1 > y_extent)) {
        exceeded = true;
    }

    return exceeded;

}

// Determines if a coordinate is outside the grid
bool BuildGrid::checkExtents(vector<float> coord, int x_extent,
                    int y_extent, float d){

    bool exceeded = false;

    // The triangle of 3 nearest nodes is defined here as a "trident"!!might pass indices to preven recalculation
    vector<vector<int>> trident = find_trident(coord, d);

    // Coordinates on the coordinate frame axes are stops calculation
    if (trident[0][0] <= 0) {
        cout << "Location left of S-Space frame extents." << endl;
        exceeded = true;

    } else if ((trident[2][1] < 0) ||
               (trident[0][1] < 0)) {
        cout << "Location beneath S-Space frame extents." << endl;
        exceeded = true;

    } else if (trident[1][0] + 1 >= x_extent) {
        cout << "Location right of S-Space frame extents." << endl;
        exceeded = true;

    } else if ((trident[0][1] + 1 > y_extent) ||   
               (trident[2][1] + 1 > y_extent)) {
        cout << "Location above of S-Space frame extents." << endl;
        exceeded = true;
    }
    return exceeded;
}