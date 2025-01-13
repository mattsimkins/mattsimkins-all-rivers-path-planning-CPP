#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include "file_handling.h"

using namespace std;

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

void save_model(BuildGrid::GridInfo* model){

    ModelFile model2write; // Initialize structure before writing to binary

    cout << sizeof(model2write) << endl;

    model2write.d = model->d;
    model2write.average_path_length = model->average_path_length;
    model2write.grid_update_count = model->grid_update_count;
    model2write.max_coord_count = model->max_coord_count;
    model2write.shortest_segment = model->shortest_segment;
    model2write.shift = model->shift;
    model2write.grid = model->grid;

    cout << sizeof(model2write) << endl;

    cout << "model size is :" << model->grid.size() << endl;
    cout << "model2write.max_coord_count: " << model->max_coord_count << endl;
    cout << "model2write.average_path_length: " << model->average_path_length << endl;
    cout << "model2write.shortest_segment: " << model->shortest_segment << endl;
    cout << "model2write.grid_update_count: " << model->grid_update_count << endl;
    cout << "model2write.d: " << model->d << endl;
    cout << "element in grid, [10][10][0] is " << model->grid[10][10][0] << endl;
    cout << "model2write.shift: " << model->shift[0] << " " << model->shift[1] << endl;

    ofstream outfile("model.bin", ios::binary);

    if (!outfile.is_open()) {
        cerr << "Error opening file!" << endl;
    }

    outfile.write(reinterpret_cast<char*>(&model2write), sizeof(ModelFile));
}

BuildGrid::GridInfo* read_model() {

    BuildGrid::GridInfo* model = new BuildGrid::GridInfo; // !!Assess memory usage with this

    ifstream file("model.bin", ios::binary); // Open the binary file

    if (!file.is_open()) {
        cerr << "Error opening file" << std::endl;
    }
 
    file.read(reinterpret_cast<char*>(model), sizeof(ModelFile)); // Read data into 'data' struct
    file.close();

    cout << "Read-in model d: " << model->d << endl;

    return model;
}

