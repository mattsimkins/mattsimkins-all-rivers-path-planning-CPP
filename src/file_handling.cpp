#include <iostream>
#include <stdio.h>
#include <sstream>
#include "file_handling.h"

using namespace std;

// Requires a directory named "training" be created in working directory
// Reads in trajectory from text file
vector<vector<float>> read_traj(string file_name) {
    string path = "../training/" + file_name;
    ifstream inputFile(path);
    vector<vector<float>> traj(0, vector<float>(2, 0.0));
    if (inputFile.is_open()) {
        std::string line;

        // Read the file line by line
        while (std::getline(inputFile, line)) {            
            stringstream ss(line);
            string word;
            string x_value = "";
            string y_value = "";

            while (getline(ss, word, ' ')) { // Use space as delimiter
                if (x_value != ""){
                    y_value = word;
                }
                if (x_value == ""){
                    x_value =  word;
                }
            }
            std::vector<float> newRow = {stof(x_value), stof(y_value)};
            traj.push_back(newRow); // Append the new row to the matrix
        }

        inputFile.close();
    } else {
        cerr << "Error: Unable to open file." << endl;
    }
    return traj;
}

// Requires a directory named "outputs" be created in working directory
void write_traj(vector<vector<float>>& traj, string file_name){
    string path = "../outputs/" + file_name;
    ofstream outfile(path);

    if (!outfile.is_open()) {
        cerr << "Error opening file!" << endl;\
    }

    for (const auto &row : traj) {
        for (const auto &element : row) {
            outfile << element << " ";
        }
        outfile << endl;
    }

    outfile.close();
}

// Requires a directory named "outputs" be created in working directory
void save_model(BuildGrid::GridInfo* model_ptr, string file_name){
    
    // Copy to type Model_R_W and martial vectors for write 
    Model_R_W model_w;
    model_w.d = model_ptr->d;
    model_w.average_path_length = model_ptr->average_path_length;
    model_w.grid_update_count = model_ptr->grid_update_count;
    model_w.max_coord_count = model_ptr->max_coord_count;
    model_w.shortest_segment = model_ptr->shortest_segment;
    model_w.start_1 = model_ptr->last_start_pt[0];
    model_w.start_2 = model_ptr->last_start_pt[1];
    model_w.shift_1 = model_ptr->shift[0];
    model_w.shift_2 = model_ptr->shift[1];
    model_w.size_i = model_ptr->grid.size();
    model_w.size_j = model_ptr->grid[0].size();
    model_w.size_k = model_ptr->grid[0][0].size();
    
    // String indexing for grid
    string str_val;
    string marshaled_vec;
    for (int i = 0; i < model_w.size_i; ++i){
        for (int j = 0; j < model_w.size_j; ++j) {
            for (int k = 0; k < model_w.size_k; ++k){
                marshaled_vec += " "; // Indices separator
                str_val = to_string(model_ptr->grid[i][j][k]);
                marshaled_vec += str_val;
            }
        }

    }
    model_w.grid_string = marshaled_vec;
    
    string path = "../outputs/" + file_name;
    ofstream outfile(path);

  if (!outfile.is_open()) {
        cerr << "Error opening file!" << endl;
    }

    outfile << model_w.d << " " 
            << model_w.average_path_length << " "
            << model_w.grid_update_count << " "
            << model_w.max_coord_count << " "
            << model_w.shortest_segment << " "
            << model_w.start_1 << " "
            << model_w.start_2 << " "
            << model_w.shift_1 << " "
            << model_w.shift_2 << " "
            << model_w.size_i << " "
            << model_w.size_j << " "
            << model_w.size_k << " "
            << model_w.grid_string << endl;
    outfile.close();
}

// Requires a directory named "outputs" be created in working directory
BuildGrid::GridInfo* read_model(string file_name) {

    BuildGrid::GridInfo* model = new BuildGrid::GridInfo;

    Model_R_W model_r; // Structure used for reading

    string path = "../outputs/" + file_name;
    std::ifstream infile(path);
    model_r.loadFromFile(infile);

    // Vector to populate from string in saved grid
    vector<vector<vector<float>>> marsheled_grid(
                                        model_r.size_i, vector<vector<float>>(
                                        model_r.size_j, vector<float>(
                                        model_r.size_k, 0.0)));
    infile.close();

    // Populate grid
    bool end;
    int l = 0;
    float num;
    string value;
    for (int i = 0; i < model_r.size_i; ++i){
        for (int j = 0; j < model_r.size_j; ++j) {
            for (int k = 0; k < model_r.size_k; ++k){
                end = false;
                while (end != true){
                    ++l;
                    if (model_r.grid_string[l] != ' ') {
                        value += model_r.grid_string[l];
                    } else {
                        end = true;
                        num = stof(value);
                        marsheled_grid[i][j][k] = num;
                    }
                }
                value = "";
            }
        }
    }

    model->d = model_r.d;
    model->average_path_length = model_r.average_path_length;
    model->grid_update_count = model_r.grid_update_count;
    model->max_coord_count = model_r.max_coord_count;
    model->shortest_segment = model_r.shortest_segment;
    model->last_start_pt = {model_r.start_1, model_r.start_2};
    model->shift = {model_r.shift_1, model_r.shift_2};
    model->grid = marsheled_grid;

    return model;
}

