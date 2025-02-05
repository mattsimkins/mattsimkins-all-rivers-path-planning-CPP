#ifndef FILE_HANDLING 
#define FILE_HANDLING
#include <string>
#include <vector>
#include <fstream>
#include "build_grid.h"

using namespace std;

    /**
    * @brief Read in a trajectory from .txt file 
    * 
    * The text file being read in must be of the numeric format
    * <x0>, <y0>
    * <x1>, <y1>
    * ...
    * <xn>, <yn>
    * where n is the number of coordinates in the text file. 
    * @param file_name Name of trajectory file, "<name>.txt".
    * @return An nx2 vector of n coordinates. Coordinates consist of an x and
    *  y location in c-space.
    */
vector<vector<float>> read_traj(string file_name);

    /**
    * @brief Write a trajectory to a .txt file. 
    * 
    * Looks for a file of the same name within a directory named "input". In
    * this version, the input directory is located in the current working
    * directory.
    * The text file being read in must be of the numeric format
    * <x0>, <y0>
    * <x1>, <y1>
    * ...
    * <xn>, <yn>
    * where n is the number of coordinates in the text file. 
    * @param file_name Name of trajectory file, "<name>.txt".
    * @return An nx2 vector of n coordinates. Coordinates consist of an x and
    *  y location in c-space.
    */
void write_traj(vector<vector<float>>& traj, string file_name); 

    /**
    * @brief Write a trajectory to a .txt file. 
    * 
    * Model is saved to a directory call "output". In this version, the output
    * directory is located in the current working directory.
    * Save model, vectors must be marshaled as strings within a structure.
    * During testing, it was found that saving a structure containing vectors
    * to a file is problematic due to padding. While not as compact in size,
    * the model is saved in a human readable form as a .txt file.
    * @param trained_model The model is potentially large, so this is passed
    */
void save_model(BuildGrid::GridInfo* trained_model, string file_name);

    /**
    * @brief Read a trajectory from a .txt file. 
    * 
    * Model is read from a directory called "output". In this version, the 
    * output directory is located in the current working directory.
    * Vectors must be marshaled as strings within a structure. This
    * data structure is not suitable for training or path estimation. Thus,
    * this serves as a wrapper that converts a format suitable for storage, and
    * converts it to a formation suitable for use with the All Rivers
    * algorithm.
    * @param trained_model The model is potentially large, so this is passed
    */
BuildGrid::GridInfo* read_model(string file_name);

/**
 *  @brief For reading and writting model to a file.
 * 
 * This structure is suitable for saving and reading trajectories. However,
 * it is not suitable for training or path estimation. Thus, this serves
 * as a wrapper for the All Rivers algorithm.
 */ 
struct Model_R_W {
    float d;
    float average_path_length;
    int grid_update_count;
    int max_coord_count;
    float shortest_segment;

    /// @brief Start and shift vectors are stored as floats.
    float start_1;
    float start_2;
    float shift_1;
    float shift_2;

    /// @brief Vector grid size varies, save dimensions for reading
    int size_i;
    int size_j;
    int size_k;
    string grid_string;
    
    /// @brief Initialize structure.
    void loadFromFile(ifstream &infile) {
        infile >> d;
        infile >> average_path_length;
        infile >> grid_update_count;
        infile >> max_coord_count;
        infile >> shortest_segment;

        /// @brief Two elements for last_start_pt vector
        infile >> start_1;
        infile >> start_2;

        /// @brief Two elements for shift vector
        infile >> shift_1;
        infile >> shift_2;

        /// @brief Not part of model. For read purposes of variable sized grid.
        infile >> size_i;
        infile >> size_j;
        infile >> size_k;

        /// @brief Included to read in variable sized grid.
        infile.ignore(1, ' ');
        getline(infile, grid_string);
    }
};

#endif