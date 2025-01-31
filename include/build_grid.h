#ifndef BUILD_GRID
#define BUILD_GRID
#include <vector>
#include <cmath>

using namespace std;

/// @brief Height of an equilateral triangle
const float Y_TRI = sqrt(3)/2;
/// @brief Max angle difference between new and old vector
const float ANGLE_THRESHOLD = 40*M_PI/180;

/**
 * @class BuildGrid
 * @brief Creates and trains a model based on training trajectories.
 * 
 * This class creates new training models by passing it a training trajectory 
 * and grid spacing. Thereafter, additional trajectories are passed to refine,
 * or enlarge the space spanned by new trajectories. The model does not enlarge
 * as it is trained on ever more training trajectories.
 * The class also accepts previously trained models so that additional training
 * is possible using an older, saved model.
 */
class BuildGrid {
public:

    /**
     * @brief For non-member functions to access public functions.
     */
    BuildGrid();

    /**
     * @brief Captures essential model information for path estimation.
     * 
     * This structure holds the x and y coordinates of a point.
     */
    struct GridInfo {
        float d; /// Node spacing
        float average_path_length; /// Average length of training paths
        int grid_update_count; /// Number of times grid has been updated
        int max_coord_count; /// Max coordinate count in training trajectories
        float shortest_segment; /// Min distance between trajectory coordinates
        vector<float> last_start_pt; /// Retains a viable start point for model
        vector<float> shift; /// Shifts C-Space to S-Space
        vector<vector<vector<float>>> grid; /// 3D matrix to store grid values
    };
    
    /**
     * @brief Constructs object using an existing model.
     *
     * This constructor is for cases where a model was trained previously,
     * saved, and reloaded for additional training.
     * @param trained_model Pointer to the trained model.
     */
    BuildGrid(GridInfo* trained_model);
    
    /**
     * @brief Zero argument constructor for accessing public functions.
     *
     * This constructor is used for a new model. The grid spacing argument
     * for this constructor is fixed for subsequent training. If a new model
     * is desired using different grid spacing, trajectory training must be
     * repeated. The trajectory arguement is used to initialize the new model's
     * s-space grid extents, and the constructor requires this first training
     * trajectory.
     * @param traj_c First training trajectory for new model.
     * @param d Grid spacing for new model.
     */
    BuildGrid(vector<vector<float>>& traj_c, float& d);
    
    /**
     * @brief Trains the grid on an existing trajectory.
     *
     * After a training model is created, or loaded from storage, this function
     * is used to train on a new, or potentially previous training trajectory.
     * @param traj The trajectory to train on.
     */
    void trainOnTrajectory(vector<vector<float>>& traj);

    /**
     * @brief A getter function for the trained model.
     * 
     * After training, this getter function obtains the training model. This
     * model can be saved, or passed to find_path in order to estimate a
     * trajectory.
     * @return returns a pointer to the trained model.
     */
    GridInfo* getModel();

    /**
     * @brief A getter function for a valid start point.
     * 
     * This function assumes that the model was previously trained on one or
     * more training trajectories.
     * When using find_path() a start point must be specified. If a start point
     * is specified that is farther than one node spacing (d) from any of the
     * the training trajectories, that start location is outside of the model
     * and an error will result. Because it is uncertain which points are
     * located near a training trajectory, the last start point of the most
     * recently learned training trajectory is retained as part of the model.
     * Strictly speaking, that start point isn't essential for the model to be
     * valid. However, retaining the start point does simplify use.
     * Importantly, trajectory estimation need not use this point. If a valid
     * start point is used at any location visited during training, find_path()
     * can estimate an alternative path.
     * @return Returns the start coordinate from the last training trajectory.
     */
    vector<float> getStartPoint();

    /**
     * @brief Performs vector addition using the standard library.
     * 
     * @param v1, v2 First and second vector to add 
     * @return The vector sum of v1 and v2.
     */
    vector<float> add_vectors(vector<float> v1, vector<float> v2);

    /**
     * @brief Performs vector subtraction using the standard library.
     * 
     * @param v1, v2 First and second vector for subtraction 
     * @return Subtracts v2 from v1.
     */
    vector<float> subtract_vectors(vector<float> v1, vector<float> v2);

    /**
     * @brief Calculates the Euclidean distance between two vectors.
     * 
     * This operation is simlar to vector subtraction, however, the length
     * of the vector is returned. If the length of a single vector is
     * desired, pass v1 and set v2 to {0, 0}
     * @param v1, v2 First and second vector 
     * @return Distance between v1 and v2.
     */
    float norm(vector<float> v1, vector<float> v2);

    /**
     * @brief Calculate scalar multiply of vector.
     * 
     * Multiplies the components of a vector by a scalar value.
     * @param v Vector
     * @param scalar Scalar to multipy the vector by.
     * @return Scaled vector.
     */
    vector<float> scalar_multiply(vector<float> v, float scalar);

    /**
     * @brief Calculate distance from location, to node given by node indices.
     * 
     * This function calculates the distance from a coordinate in space to the
     * coordinate of a node. The nodes index maps to a coordinate.
     * @param loc Location in Euclidean space.
     * @param indices Indices of grid node.
     * @param d Model grid spacing.
     * @return Euclidean distance (vector norm).
     */
    float dist2node(vector<float> loc, vector<int> indices, float d);

    /**
     * @brief Calculates the coordinates of a node given its index
     * 
     * @param indices Indices of grid node.
     * @param d Model grid spacing.
     * @return Euclidean distance (vector norm).
     */
    vector<float> index2coord(vector<int> index, float d);

    
        /**
     * @brief Calculates the coordinates of a node given its index
     * 
     * This function finds the three closest nodes to the given coordinate. In
     * other words, it finds the triangle, or "trident" of nodes that form an
     * equallateral triangle, or "sextant", that circumscribes the coordinate.
     * This function is used to gather the three nodes that are updated with
     * vectors during training. It is also used to find the three nodes to use
     * for interpolation during path estimation, see find_path.h.
     * @param coord Indices of grid node.
     * @param d Model grid spacing.
     * @return A 3x2 vector where column 2 is the index of a nodes along the
     * x-direction in s-space, column 2 is the index along the y-direction. The
     * first row corresponds to the left node, the second row to the right
     * node, and the third row to the center node.
     */
    vector<vector<int>> find_trident(vector<float> coord, float d);

    /**
     * @brief Resizes grid if if training trajectory exceeds extents.
     * 
     * Because the grid of nodes must span s-space, the size of that space is
     * necessarily bounded due to memory constraints. In this way, the space
     * complexity is proportional to x_extent*y_extent. If a training
     * trajectory ventures outside this bound, the s-space extents must
     * enlarge. If the next calculated point in an estimated trajectory
     * exceeds this bound, the estimated trajectory must terminate.
     * @param coord Indices of grid node.
     * @param x_extent Largest x in bounded s-space frame.
     * @param y_extent Largest y in bounded s-space frame.
     * @param d Model grid spacing.
     * @return True for the coordinate being outside extents, false for the
     * coordinate being within extents 
     */
    bool outsideExtents(vector<float> coord, int x_extent,
                        int y_extent, float d);

    /**
     * @brief Determines if an estimated coordinate is outside the grid.
     * 
     * S-space must accomodate trajectories used for training, see
     * outsideExtents(). However, if an *estimated* trajectory ventures outside
     * those constraints the grid is not resized. Instead, further growth
     * of the estimated trajectory is assumed invalid as it would extend
     * beyond the extent of all training trajectories. In that case, the
     * estimated trajectory calculation must cease based on the return
     * value. However, if the calculation simply nears the node location that
     * is outiside the extents, the calculation may proceed minus that node.
     * @param coord Indices of grid node.
     * @param x_extent Largest x in bounded s-space frame.
     * @param y_extent Largest y in bounded s-space frame.
     * @param d Model grid spacing.
     * @return True for the coordinate being outside extents, false for the
     * coordinate being within extents 
     */
    bool checkExtents(vector<float> coord, int x_extent,
                        int y_extent, float d);

    /**
     * @brief Finds the index of the closest node to the given coordinate
     * 
     * Finds the single node nearest the given coordinate.
     * @param coord Location of interest coordinate.
     * @param d Model grid spacing.
     * @return Two element vector containing the indices of the node
     */
    vector<int> find_closest_index(vector<float> coord, float d);

    /**
     * @brief Find S-Space coordinates given index
     * 
     * @param ind Indices of grid node.
     * @param d Model grid spacing.
     * @return Two element vector containing the indices of the node
     */
    vector<float> coord_from_ind(vector<int> ind, float d);

    /**
     * @brief Trajectory metrics are for later use when estimating a path from
     * grid.
     * 
     * The structure metrics is returned from trajectory_metrics. These metrics
     * are usefull for calculating an estimated path, see find_path. In
     * particular, when find_path estimates a trajectory based on nodes,
     * determining when the calculation should stop is non-obvious. Without
     * a stop condition find_path gets trapped in an infinite loop. In
     * practice, this manefests itself as a tiny tail at the end of the
     * estimated path consisting of infintesimal segments. These metrics are
     * used to end estimation. Path length prevents the estimated path from
     * growing past the average length of training pathes. Shortest segment
     * ends estimation if the newly added segement is significantly smaller
     * than the smallest segment encountered in training. Finally, coord_count
     * ensures that the estimated trajectory does not exceed the maximum
     * number of coordinates encounted in training trajectories.
     * @param path_length Overall length of the trajectory passed.
     * @param shortest_segment Shortest trajectory segment encountered in all
     * training trajectories.
     * @param coord_count The largest coordinate count encountered in all
     * training trajectories
     */
    struct metrics {
        float path_length;
        float shortest_segment;
        float coord_count;
    };
    metrics trajectory_metrics(vector<vector<float>> traj);


    ~BuildGrid();


private:

    /**
     * @brief Removes consecutive duplicate points from trajectory
     * 
     * This applies to training trajectories. If a training trajectory contains
     * duplicate consequtive coordinates (i.e. stalls) then the trained vector
     * value for the nearby nodes becomes zero. This will cause an error.
     * Therefore, the trajectory is scanned for duplicates. Rather than
     * generate an error, the duplicate is removed before further processing.
     */
    void fixDuplicates();

    /**
     * @brief Constructs a grid based on the first training trajectory.
     * 
     * Constructs a new grid based on trajectory and grid spacing arguments.
     * Thereafter, this is used to create larger grids for cases where a
     * a training trajectory is too large for the previous grid.
     */
    void initializeGrid(); // Initializes the grid for first training trajectory

    /**
     * @brief Trains the grid with the trajectory in c-space.
     * 
     * Used after the BuildGrid constructor was invoked. This is used to train
     * on additional training trajectories thereafter.
     */
    void trainGrid(const vector<vector<float>>& traj);

    /**
     * @brief Resizes grid if a training trajectory exceeds old extents
     * 
     * This preempts other extents checks by doing an early evaluation of the
     * training trajectory before further processing is attempted.
     */
    void checkExtents_c();

    /**
     * @brief Updates an individual node with vectors.
     * 
     * If a node has a zero length vector, the node is overwritten. If the node
     * was visited previously during training, the update evaluates the
     * previous vector value. If the previous vector is in a similar direction
     * to the update, as defined by ANGLE_THRESHOLD, then the update includes
     * the previous values direction and magnitude in the update. However, in
     * cases where ANGLE_THRESHOLD is exceeded, the previous value is assumed
     * to point in a direction contrary to the update. In that case, the old
     * vector values are entirely overwritten. This is intended to overcome the
     * problem of back-tracking. To use a maze dead end as an example, if a
     * training trajector reach a dead end, the path must reverse direction and
     * back-track. Overwritting the old value helps ensure that path estimation
     * does not route to a dead end.
     * @param vec Vector to update a node with
     * @param indices Indices of node within grid
     */
    void update_node(const vector<float>& vec, const vector<int>& indices);

    /**
     * @brief Calculates C-Space extents of trajectory
     * 
     * Evaluates the extents in the difference between the extenst needed to
     * accomodate a new training trajectory and the existing extents.
     * @return A 4 element vector describeing the minimum x, maximum x,
     * minimum y, and maximum y respectively.
     */
    vector<float> calculateExtents_c();

    GridInfo model; // Declares the Grid info structure

    /**
     * @brief Initializes a grid based on the number of rows, columns, and
     * depth needed.
     * 
     * Initializes a grid. The number of rows and columns are predetermined.
     * @param rows The number of indices along x in s-space.
     * @param cols The number of indices along y in s-space.
     * @param depth For the 2D case this is equal to two.
     * @return A 3D vector descibing the training grid of nodes.
     */
    vector<vector<vector<float>>> sizeGrid(const int& rows, 
                                           const int& cols,
                                           const int& depth);
    
    /// @brief Coordinate frame extents in C-Space.
    vector<float> cSpaceExtents;
    
    /// @brief Not using model.d so that public member functions are accessable.
    float d;
    
    /// @brief C-space "configuration space" trajectory.
    vector<vector<float>> traj_c;
    
    /// @brief Transformed to s-space trajectory.
    vector<vector<float>> traj_s;
};

#endif