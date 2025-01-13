#ifndef PATH_FUNCTIONS_H
#define PATH_FUNCTIONS_H

#include <cmath>
#include <vector>
using namespace std;

const float Y_TRI = sqrt(3)/2; // Height of an equilateral triangle
const float SHORTEST_SEGMENT_MARGIN = 50; // Percentage of shortest trajectory segment to add to grid
const float POINT_COUNT_MARGIN = 50; // Percentage of trajectory point count to add to grid
const float ANGLE_THRESHOLD = 40*M_PI/180; // Maximum angle difference between new and old vector

// Adds two vectors element-wise
vector<float> add_vectors(const vector<float>& v1, const vector<float>& v2);

// Subtracts two vectors element-wise
vector<float> subtract_vectors(const vector<float>& v1, const vector<float>& v2);

// Calculates the Euclidean distance between two vectors
float norm(const vector<float>& v1, const vector<float>& v2);

// Calculate scalar multiply of vector
vector<float> scalar_multiply(const vector<float>& v, const float& scalar);

// Calculate distance between a location, and the coordinate of a node given by its indices
float dist2node(const vector<float>& loc, const vector<int>& indices, const float& d);

// Calculates the coordinates of a node given its index
vector<float> index2coord(const vector<int>& index, const float& d);

// Finds the three closest nodes to the given coordinate
vector<vector<int>> find_trident(const vector<float>& coord, const float& d);

// Determines if a coordinate is outside the grid
bool outsideExtents(const vector<float>& coord, const int& x_extent,
                    const int& y_extent, const float& d);

// Throws error if a coordinate is outside the grid
bool checkExtents(const vector<float>& coord, const int& x_extent,
                    const int& y_extent, const float& d);

// Finds the index of the closest node to the given coordinate
vector<int> find_closest_index(const vector<float>& coord, const float& d);

// Find S-Space coordinates given index
vector<float> coord_from_ind(const vector<int>& ind, const float& d);

// Trajectory metrics are for later use when estimating a path from grid
struct metrics {
    float path_length;
    float shortest_segment;
    float coord_count;
};
metrics trajectory_metrics(const vector<vector<float>>& traj);

#endif // PATH_FUNCTIONS_H