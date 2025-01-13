#include <iostream>
#include <cmath> 
#include <vector>
#include <limits>
#include <cmath>
#include "path_functions.h"

using namespace std;


// Adds two vectors element-wise
vector<float> add_vectors(const vector<float>& v1, const vector<float>& v2) {
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
vector<float> subtract_vectors(const vector<float>& v1,
                               const vector<float>& v2) {
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
float norm(const std::vector<float>& v1, const std::vector<float>& v2) {
    float sum = 0;
    for (int i = 0; i < v1.size(); i++) {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }
    return sqrt(sum);
}

vector<float> scalar_multiply(const vector<float>& v, const float& scalar){
    vector<float> result(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * scalar;
    }
    return result;
}

vector<float> index2coord(const vector<int>& index, const float& d){
    vector<float> coord = {(float)index[0] * d, (float)index[1] * d * Y_TRI};
    if (index[1] % 2 != 0) {
        coord[0] += d / 2;
    }
    return coord;
}

vector<int> find_closest_index(const vector<float>& coord, const float& d){
    int j_closest = round(coord[1] / (d * Y_TRI));
    int i_closest = round(coord[0] / d - 0.5);
    if (j_closest % 2 == 0) {
        int i_closest = round(coord[0] / d);
    }
    return {i_closest, j_closest};
}

vector<float> coord_from_ind(const vector<int>& ind, const float& d){
    float x;
    if (ind[1] % 2 == 0) {
            x = ind[0]*d;
        } else { 
            x = ind[0]*d + d/2;
        }
    
    float y = ind[1]*d*Y_TRI;
    return {x, y};
}

float dist2node(const vector<float>& loc, const vector<int>& indices, const float& d){
    vector<float> node_coord = index2coord(indices, d);
    return norm(loc, node_coord);
}

metrics trajectory_metrics(const vector<vector<float>>& traj){
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

// Returns 3 rows of indices, to the left, right, and center of coordinate
vector<vector<int>> find_trident(const vector<float>& coord, const float& d) {

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

// Determines if a coordinate is outside the grid
bool outsideExtents(const vector<float>& coord, const int& x_extent,
                    const int& y_extent, const float& d){

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
bool checkExtents(const vector<float>& coord, const int& x_extent,
                    const int& y_extent, const float& d){

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