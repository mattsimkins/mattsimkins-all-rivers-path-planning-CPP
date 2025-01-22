#ifndef BUILD_GRID
#define BUILD_GRID
#include <vector>

using namespace std;

/*
This class estimates trajectories from a grid. Importantly, it does not just
use the grid to estimate a path, it also updates the grid with new, calculated
trajectories. Therefore, there's arguably a weak dependency between the 
BuildGrid and FindPath classes because FindPath has some elements of training.
*/

class BuildGrid {
public:

    // Constructor to develop for trained grid and trajectory as aguments
    BuildGrid(vector<vector<float>>& traj_c);
    
    // Constructor for new grid
    BuildGrid(vector<vector<float>>& traj_c, float& d);
    
    // Training on existing grid
    void trainOnTrajectory(vector<vector<float>>& traj);
    
    // Captures essential training model information
    struct GridInfo {
        float d; // Node spacing
        float average_path_length; // Average length of training paths
        int grid_update_count; // Number of times grid has been updated
        int max_coord_count; // Max coordinate count in training trajectories
        float shortest_segment;
        vector<float> last_start_pt;
        vector<float> shift; //Shifts C-Space to S-Space
        vector<vector<vector<float>>> grid; // 3D matrix to store grid values
    };

    GridInfo* getModel(); // Member function to return model pointer
    vector<float> getStartPoint(); // Retains a valid start point

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
    int grid_update_count; // Number of times grid has been updated
    float average_path_length;
    int max_coord_count;
    float shortest_segment;
    vector<float> sSpaceExtents;
    float d;
    float gridSizeX;
    float gridSizeY;
    float variable;
    vector<vector<float>> traj_c;
    vector<vector<float>> traj_s;
};

#endif