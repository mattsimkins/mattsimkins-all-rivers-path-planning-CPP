#ifndef BUILD_GRID
#define BUILD_GRID
#include <vector>
#include <cmath>

using namespace std;

const float Y_TRI = sqrt(3)/2; // Height of an equilateral triangle
const float ANGLE_THRESHOLD = 40*M_PI/180; // Maximum angle difference between new and old vector



/*
This class estimates trajectories from a grid. Importantly, it does not just
use the grid to estimate a path, it also updates the grid with new, calculated
trajectories. Therefore, there's arguably a weak dependency between the 
BuildGrid and FindPath classes because FindPath has some elements of training.
*/


/**
 * @class BuildGrid
 * @brief Creates and trains a model based on training trajectories.
 */
class BuildGrid {
public:

    /**
     * @brief Zero argument constructor for accessing public functions.
     */
    BuildGrid();

    // Captures essential training model information
    struct GridInfo {
        float d; // Node spacing
        float average_path_length; // Average length of training paths
        int grid_update_count; // Number of times grid has been updated
        int max_coord_count; // Max coordinate count in training trajectories
        float shortest_segment; // Min distance between trajectory coordinates
        vector<float> last_start_pt; // Retains a viable start point for model
        vector<float> shift; // Shifts C-Space to S-Space
        vector<vector<vector<float>>> grid; // 3D matrix to store grid values
    };
    
    BuildGrid(GridInfo* trained_model);
    
    /**
     * @brief Zero argument constructor for accessing public functions.
     *
     * @param w The width of the rectangle.
     * @param h The height of the rectangle.
     */
    // Constructor for new grid
    BuildGrid(vector<vector<float>>& traj_c, float& d);
    
    // Training on existing grid
    void trainOnTrajectory(vector<vector<float>>& traj);

    GridInfo* getModel(); // Member function to return model pointer
    vector<float> getStartPoint(); // Retains a valid start point

    //Adds two vectors element-wise
    vector<float> add_vectors(vector<float> v1, vector<float> v2);

    // Subtracts two vectors element-wise
    vector<float> subtract_vectors(vector<float> v1, vector<float> v2);

    // Calculates the Euclidean distance between two vectors
    float norm(vector<float> v1, vector<float> v2);

    // Calculate scalar multiply of vector
    vector<float> scalar_multiply(vector<float> v, float scalar);

    // Calculate distance between a location, and the coordinate of a node given by its indices
    float dist2node(vector<float> loc, vector<int> indices, float d);

    // Calculates the coordinates of a node given its index
    vector<float> index2coord(vector<int> index, float d);

    // Finds the three closest nodes to the given coordinate
    vector<vector<int>> find_trident(vector<float> coord, float d);

    // Determines if a coordinate is outside the grid
    bool outsideExtents(vector<float> coord, int x_extent,
                        int y_extent, float d);

    // Throws error if a coordinate is outside the grid
    bool checkExtents(vector<float> coord, int x_extent,
                        int y_extent, float d);

    // Finds the index of the closest node to the given coordinate
    vector<int> find_closest_index(vector<float> coord, float d);

    // Find S-Space coordinates given index
    vector<float> coord_from_ind(vector<int> ind, float d);

    // Trajectory metrics are for later use when estimating a path from grid
    struct metrics {
        float path_length;
        float shortest_segment;
        float coord_count;
    };
    metrics trajectory_metrics(vector<vector<float>> traj);


    ~BuildGrid();


private:
    void fixDuplicates(); // Removes consecutive duplicate points from trajectory
    void initializeGrid(); // Initializes the grid for first training trajectory
    void trainGrid(const vector<vector<float>>& traj); // Trains the grid with the trajectory in c-space
    void checkExtents_c(); // Resizes grid if a training trajectory exceeds old extents
    void updateNode(const vector<float>& vec, const vector<int>& indices);
    GridInfo model;
    //GridInfo* model_ptr = &model;
    vector<float> calculateExtents_c(); // Calculates C-Space extents of trajectory
    vector<vector<vector<float>>> sizeGrid(const int& rows, const int& cols, const int& depth); // Resizes the grid
    vector<float> cSpaceExtents; // Coordinate frame extents in C-Space
    float d; // Not using model.d so that public member functions are accessable
    vector<vector<float>> traj_c;
    vector<vector<float>> traj_s;
};

#endif