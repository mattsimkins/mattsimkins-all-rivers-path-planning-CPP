#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include "file_handling.h"

using namespace std;


// Reads in trajectory from text file
vector<vector<float>> read_traj(string file_name) {
    std::ifstream inputFile(file_name);
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
        /*
        // Print the matrix
        for (const auto &row : traj) {
            for (int val : row) {
                std::cout << val << " ";
            }
        cout << endl;
        }
        */

        inputFile.close();
    } else {
        std::cerr << "Error: Unable to open file." << std::endl;
    }
    return traj;
}

// Structure for reading and writting model to a file
struct Model_R_W {
    float d;

    float average_path_length;
    int grid_update_count;
    int max_coord_count;
    float shortest_segment;

    float shift_1;
    float shift_2;

    // Grid of variable dimensions
    int size_i;
    int size_j;
    int size_k;
    string grid_string; // 3D vector marshaled as string
    void loadFromFile(ifstream &infile) {
        infile >> d;

        infile >> average_path_length;
        infile >> grid_update_count;
        infile >> max_coord_count;
        infile >> shortest_segment;

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



void save_model(BuildGrid::GridInfo* model_ptr){
    
    // Copy to type Model_R_W to martial vectors for read/write 
    Model_R_W model_w;
    model_w.d = model_ptr->d;
    model_w.average_path_length = model_ptr->average_path_length;
    model_w.grid_update_count = model_ptr->grid_update_count;
    model_w.max_coord_count = model_ptr->max_coord_count;
    model_w.shortest_segment = model_ptr->shortest_segment;
    model_w.shift_1 = model_ptr->shift[0];
    model_w.shift_2 = model_ptr->shift[1];

    model_w.size_i = model_ptr->grid.size();
    model_w.size_j = model_ptr->grid[0].size();
    model_w.size_k = model_ptr->grid[0][0].size();
    
    // String indexing for grid
    string str_val;
    string marshaled_vec; // String containing indices and angles
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
    
    ofstream outfile("model.txt");

  if (!outfile.is_open()) {
        cerr << "Error opening file!" << endl;
    }



    outfile << model_w.d << " " 
            << model_w.average_path_length << " "
            << model_w.grid_update_count << " "
            << model_w.max_coord_count << " "
            << model_w.shortest_segment << " "
            << model_w.shift_1 << " "
            << model_w.shift_2 << " "
            << model_w.size_i << " "
            << model_w.size_j << " "
            << model_w.size_k << " "
            << model_w.grid_string << endl;
    outfile.close();

}


BuildGrid::GridInfo* read_model() {

    BuildGrid::GridInfo* model = new BuildGrid::GridInfo; // !!Assess memory usage with this

    Model_R_W model_r; // Uses read/write structure
    //BuildGrid::GridInfo model_return; // Uses structure for training and estimation

    std::ifstream infile("model.txt");
    model_r.loadFromFile(infile);

    // Growing model_return dynamically in nested loops would be more space efficient!!
    vector<vector<vector<float>>> marsheled_grid(
                                        model_r.size_i, vector<vector<float>>(
                                        model_r.size_j, vector<float>(
                                        model_r.size_k, 0.0)));
    infile.close();

    // Populate model
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
    model->shift = {model_r.shift_1, model_r.shift_2};
    model->grid = marsheled_grid; // !!not very memory efficient



    return model;
}

