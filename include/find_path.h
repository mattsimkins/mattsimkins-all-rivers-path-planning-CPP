#ifndef FIND_PATH_H
#define FIND_PATH_H

#include <vector>
#include "build_grid.h"
//#include "file_handling.h"

using namespace std;

const float SHORTEST_SEGMENT_MARGIN = 50; // Percentage of shortest trajectory segment to add to gridrid
const float POINT_COUNT_MARGIN = 50; // Percentage of trajectory point count to add to grid

/**
 * @class FindPath
 * @brief Creates and trains a model based on training trajectories.
 * 
 * This class creates new training models by passing it a training trajectory 
 * and grid spacing. Thereafter, additional trajectories are passed to refine,
 * or enlarge the space spanned by new trajectories. The model does not enlarge
 * as it is trained on ever more training trajectories.
 * The class also accepts previously trained models so that additional training
 * is possible using an older, saved model.
 */
class FindPath {
public:
    /**
     * @brief Uses an already trained model and start point to estimate a path.
     * 
     * A model pointer is passed to initialize struct model. A start point is
     * also passed. A path is then estimated from the start point to the
     * approximate end point where training trajectoies commonly ended.
     * @param trained_model A pointer to a trained model structure.
     * @param start_point Passed as a vector of an arbitrary location. The
     * start point is arbitrary in the sense that it must be on, or near, one
     * or more past training trajectory trajectories. In this sense, near is
     * meant to imply within a distance less than the grid spacing.
     */
    FindPath(BuildGrid::GridInfo* trained_model, vector<float>& start_point);
    
    /**
     * @brief Estimates a path given a new start point.
     * 
     * After the FindPath object is created this function allows it to
     * estimate a path given other start points. These points need not be
     * located on, or near previously used start points.
     * @return 2D vector of an estimated trajectory given by two columns of
     * x-y coordinates and m rows of coordinates.
     */
    void find_path(vector<float>& start_point);

    /**
     * @brief Getter function for the estimated a path.
     * 
     * This trajectory becomes available after the FindPath constuctor is
     * invoked, or when the find_path function is invoked using a new start
     * point with the same model.
     * @return 2D vector of an estimated trajectory given by two columns of
     * x-y coordinates and m rows of coordinates.
     */
    vector<vector<float>> get_path();
    
    ~FindPath();

private:
    /**
     * @brief Declares a BuildGrid object to make member functions accessable.
     * 
     * BuildGrid and FindPath use common functions. Rather than have the two
     * classes share a common cpp file of functions, FindPath accesses
     * function definitions in BuildGrid.
     */
    BuildGrid bg;

    /**
     * @brief Triggers the calculation of an estimated path.
     */
    void calculate_path(); // Calculates path based on trained model

    /**
    * @brief Estimates next coordinate in path when training model only
    * includes one non-zero node.
    * 
    * This function is invoked during path estimation. It sometimes occurs
    * that an estimated path encounters a cluster of nodes (3 nodes) wherein
    * only one node was visited during training, and the other two nodes
    * contain zero vectors. In this case, interpolation unecessary because
    * the estimate only has one vector to work from. That vector is used to
    * calculate the next location along the estimated trajectory. Moreover,
    * the empty nodes are populated with vectors that point to the same
    * location as the next estimated coordinate. This is accomplished by 
    * invocation of update_empty_node. In this way, path estimation is used to
    * provide further grid training. However, this case is rarely encountered.
    * @param loc Location of the estimated trajectory befor the next
    * coordinate is calculated.
    * @param triad_vecs A 3x2 vector of node indices.
    * @param visited_nodes A 3 element vector where each element cooresponds
    * to the state of the three nearest nodes to the left, right, and center
    * respectively of loc. True indicates that the node had been visited
    * during training and has a non-zero vector. False indicates that the
    * node is unvisited and has a zero vector.
    * @param indices A 3x2 vector of grid indices. The first column is a
    * first index, the second is a second index. The rows correspond to
    * the node location relative to loc, again, to the left, right, and
    * center respectively.
    */
    vector<float> two_empty_nodes(const vector<float>& loc,
                                  const vector<vector<float>>& triad_vecs,
                                  const vector<bool>& visited_nodes,
                                  const vector<vector<int>>& indices);

    /**
    * @brief Estimates next coordinate in path when training model only
    * includes two non-zero nodes.
    * 
    * This function is invoked during path estimation. It sometimes occurs
    * that an estimated path encounters a cluster of nodes (3 nodes) wherein
    * only two of three nodes was visited during training. In this case, 
    * interpolation must take an average of distances. These distances are
    * from location loc to the location of the nodes Moreover, the empty node
    * is populated with vectors that point to the same location as the next 
    * estimated coordinate by invoking update_empty_node. In this way, path
    * estimation is used to provide further grid training. However, this case
    * is infrequently encountered.
    * @param loc Location of the estimated trajectory befor the next
    * coordinate is calculated.
    * @param triad_vecs A 3x2 vector of node indices.
    * @param visited_nodes A 3 element vector where each element cooresponds
    * to the state of the three nearest nodes to the left, right, and center
    * respectively of loc. True indicates that the node had been visited
    * during training and has a non-zero vector. False indicates that the
    * node is unvisited and has a zero vector.
    * @param indices A 3x2 vector of grid indices. The first column is a
    * first index, the second is a second index. The rows correspond to
    * the node location relative to loc, again, to the left, right, and
    * center respectively.
    */
    vector<float> one_empty_node(const vector<float>& loc,
                                 const vector<vector<float>>& triad_vecs,
                                 const vector<bool>& visited_nodes,
                                 const vector<vector<int>>& indices);

    /**
    * @brief Estimates next coordinate in a path where all tree nodes have
    * non-zero values. This is the best and most common case. 
    * 
    * An estimated vector to the next estimated coordinate is calculated using
    * a combination of all three nodes. To do this, the three nodes are 
    * weighted based on their relative distance to the location loc.
    * @param loc Location of the estimated trajectory befor the next
    * coordinate is calculated.
    * @param triad_vecs A 3x2 vector of node indices.
    * @param visited_nodes A 3 element vector where each element cooresponds
    * to the state of the three nearest nodes to the left, right, and center
    * respectively of loc. True indicates that the node had been visited
    * during training and has a non-zero vector. False indicates that the
    * node is unvisited and has a zero vector.
    * @param indices A 3x2 vector of grid indices. The first column is a
    * first index, the second is a second index. The rows correspond to
    * the node location relative to loc, again, to the left, right, and
    * center respectively.
    */
    vector<float> no_empty_node(const vector<float>& loc,
                                const vector<vector<float>>& triad_vecs,
                                const vector<vector<int>>& indices);
    
        /**
    * @brief Populates empty nodes encountered during path estimation. 
    * 
    * Used to populate a specific node with a specific vector.
    * @param vec 1x2 vector to update a node with.
    * @param index 1x2 vector of grid indices for the node.
    */
    void update_empty_node(const vector<float>& vec, const vector<int>& index);

    /// @brief Starting point in c-space for estimated trajectory.
    vector<float> start_c;

    /// @brief Pointer to the model structure
    BuildGrid::GridInfo* model;

    /// @brief The desired estimated trajectory
    vector<vector<float>> calc_traj;

    /// @brief
    float running_length; // Monitoring for excessive length of calc_traj
};
#endif // FIND_PATH_H