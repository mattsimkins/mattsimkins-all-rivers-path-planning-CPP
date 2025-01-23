#ifndef FILE_HANDLING 
#define FILE_HANDLING
#include <string>
#include <vector>
#include <fstream>
#include "build_grid.h"

using namespace std;

// Read in a .txt file
vector<vector<float>> read_traj(string file_name);

// Save model, vectors must be marshaled to strings
void save_model(BuildGrid::GridInfo* trained_model, string& file_name);

// Read model, strings must be marshaled to vectors
BuildGrid::GridInfo* read_model(string& file_name);

// For reading and writting model to a file
struct Model_R_W {
    float d;
    float average_path_length;
    int grid_update_count;
    int max_coord_count;
    float shortest_segment;

    // Start and shift vectors are stored as floats
    float start_1;
    float start_2;
    float shift_1;
    float shift_2;

    // Vector grid size varies, save dimensions for reading
    int size_i;
    int size_j;
    int size_k;
    string grid_string; // 3D vector marshaled as string
    
    // Retain locations in struct
    void loadFromFile(ifstream &infile) {
        infile >> d;
        infile >> average_path_length;
        infile >> grid_update_count;
        infile >> max_coord_count;
        infile >> shortest_segment;

        // Two elements of last_start_pt vector
        infile >> start_1;
        infile >> start_2;

        // Two elements of shift vector
        infile >> shift_1;
        infile >> shift_2;

        // Not part of model, but for read/write purposes
        infile >> size_i;
        infile >> size_j;
        infile >> size_k;

        infile.ignore(1, ' ');
        getline(infile, grid_string);
    }
};

#endif