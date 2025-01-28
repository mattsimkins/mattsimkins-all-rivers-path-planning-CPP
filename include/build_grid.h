/**
 * @file build_grid.h
 * @brief Header file for the BuildGrid class which estimates and updates trajectories on a grid.
 */

/**
 * @class BuildGrid
 * @brief Creates and trains a model based on training trajectories.
 * 
 * Trains a model (grid) by iteratively reading in 2D trajectories. The model
 * includes various pieces of meta-data, as well as a 3D grid. The nodes of
 * this grid allow for path estimation, see find_path.h.
 */
class BuildGrid {
public:
    /**
     * @brief Zero argument constructor for accessing public functions.
     */
    BuildGrid();
    
    /**
     * @brief Constructor for initializing a new grid.
     * 
     * This 
     * @param traj_c Training trajectories in C-space.
     * @param d Node spacing.
     */
    BuildGrid(vector<vector<float>>& traj_c, float& d);
    
    /**
     * @brief Trains the grid on an existing trajectory.
     * 
     * @param traj Training trajectory.
     */
    void trainOnTrajectory(vector<vector<float>>& traj);
    
    /**
     * @brief Structure to capture essential training model information.
     */
    struct GridInfo {
        float d; ///< Node spacing
        float average_path_length; ///< Average length of training paths
        int grid_update_count; ///< Number of times grid has been updated
        int max_coord_count; ///< Max coordinate count in training trajectories
        float shortest_segment; ///< Min distance between trajectory coordinates
        vector<float> last_start_pt; ///< Retains a viable start point for model
        vector<float> shift; ///< Shifts C-Space to S-Space
        vector<vector<vector<float>>> grid; ///< 3D matrix to store grid values
    };

    /**
     * @brief Returns a pointer to the model.
     * 
     * @return Pointer to GridInfo structure.
     */
    GridInfo* getModel();

    /**
     * @brief Retains a valid start point.
     * 
     * @return Vector containing the start point.
     */
    vector<float> getStartPoint();

    /**
     * @brief Adds two vectors element-wise.
     * 
     * @param v1 First vector.
     * @param v2 Second vector.
     * @return Resultant vector after addition.
     */
    vector<float> add_vectors(vector<float> v1, vector<float> v2);

    /**
     * @brief Subtracts two vectors element-wise.
     * 
     * @param v1 First vector.
     * @param v2 Second vector.
     * @return Resultant vector after subtraction.
     */
    vector<float> subtract_vectors(vector<float> v1, vector<float> v2);

    /**
     * @brief Calculates the Euclidean distance between two vectors.
     * 
     * @param v1 First vector.
     * @param v2 Second vector.
     * @return Euclidean distance.
     */
    float norm(vector<float> v1, vector<float> v2);

    /**
     * @brief Calculates the scalar multiplication of a vector.
     * 
     * @param v Vector to be multiplied.
     * @param scalar Scalar value.
     * @return Resultant vector after scalar multiplication.
     */
    vector<float> scalar_multiply(vector<float> v, float scalar);

    /**
     * @brief Calculates the distance between a location and the coordinate of a node given by its indices.
     * 
     * @param loc Location vector.
     * @param indices Indices of the node.
     * @param d Node spacing.
     * @return Distance to the node.
     */
    float dist2node(vector<float> loc, vector<int> indices, float d);

    /**
     * @brief Calculates the coordinates of a node given its index.
     * 
     * @param index Index of the node.
     * @param d Node spacing.
     * @return Coordinates of the node.
     */
    vector<float> index2coord(vector<int> index, float d);

    /**
     * @brief Finds the three closest nodes to the given coordinate.
     * 
     * @param coord Coordinate vector.
     * @param d Node spacing.
     * @return Vector of indices of the three closest nodes.
     */
    vector<vector<int>> find_trident(vector<float> coord, float d);

    /**
     * @brief Determines if a coordinate is outside the grid.
     * 
     * @param coord Coordinate vector.
     * @param x_extent X extent of the grid.
     * @param y_extent Y extent of the grid.
     * @param d Node spacing.
     * @return True if the coordinate is outside the grid, false otherwise.
     */
    bool outsideExtents(vector<float> coord, int x_extent, int y_extent, float d);

    /**
     * @brief Throws an error if a coordinate is outside the grid.
     * 
     * @param coord Coordinate vector.
     * @param x_extent X extent of the grid.
     * @param y_extent Y extent of the grid.
     * @param d Node spacing.
     * @return True if the coordinate is within the grid, false otherwise.
     */
    bool checkExtents(vector<float> coord, int x_extent, int y_extent, float d);

    /**
     * @brief Finds the index of the closest node to the given coordinate.
     * 
     * @param coord Coordinate vector.
     * @param d Node spacing.
     * @return Index of the closest node.
     */
    vector<int> find_closest_index(vector<float> coord, float d);

    /**
     * @brief Finds the S-Space coordinates given an index.
     * 
     * @param ind Index vector.
     * @param d Node spacing.
     * @return S-Space coordinates.
     */
    vector<float> coord_from_ind(vector<int> ind, float d);

    /**
     * @brief Structure to capture trajectory metrics for later use when estimating a path from the grid.
     */
    struct metrics {
        float path_length; ///< Length of the path
        float shortest_segment; ///< Shortest segment in the path
        float coord_count; ///< Number of coordinates in the path
    };

    /**
     * @brief Calculates trajectory metrics.
     * 
     * @param traj Trajectory vector.
     * @return Metrics structure containing path length, shortest segment, and coordinate count.
     */
    metrics trajectory_metrics(vector<vector<float>> traj);

    /**
     * @brief Destructor for the BuildGrid class.
     */
    ~BuildGrid();

private:
    /**
     * @brief Removes consecutive duplicate points from the trajectory.
     */
    void fixDuplicates();

    /**
     * @brief Initializes the grid for the first training trajectory.
     */
    void initializeGrid();

    /**
     * @brief Trains the grid with the trajectory in C-space.
     * 
     * @param traj Training trajectory.
     */
    void trainGrid(const vector<vector<float>>& traj);

    /**
     * @brief Resizes the grid if a training trajectory exceeds old extents.
     */
    void checkExtents_c();

    /**
     * @brief Updates a node with the given vector and indices.
     * 
     * @param vec Vector to update the node with.
     * @param indices Indices of the node.
     */
    void updateNode(const vector<float>& vec, const vector<int>& indices);

    /**
     * @brief Calculates the C-Space extents of the trajectory.
     * 
     * @return Vector containing the C-Space extents.
     */
    vector<float> calculateExtents_c();

    /**
     * @brief Resizes the grid.
     * 
     * @param rows Number of rows.
     * @param cols Number of columns.
     * @param depth Depth of the grid.
     * @return Resized 3D grid.
     */
    vector<vector<vector<float>>> sizeGrid(const int& rows, const int& cols, const int& depth);

    GridInfo model; ///< Model information
    vector<float> cSpaceExtents; ///< Coordinate frame extents in C-Space
    int grid_update_count; ///< Number of times grid has been updated
    float average_path_length; ///< Average length of training paths
    int max_coord_count; ///< Max coordinate count in training trajectories
    float shortest_segment; ///< Min distance between trajectory coordinates
    vector<float> sSpaceExtents; ///< Coordinate frame extents in S-Space
    float d; ///< Node spacing
    float gridSizeX; ///< Grid size in X direction
    float gridSizeY; ///< Grid size in Y direction
    float variable; ///< Variable for internal use
    vector<vector<float>> traj_c; ///< Training trajectories in C-space
    vector<vector<float>> traj_s; ///< Training trajectories in S-space
};

#endif