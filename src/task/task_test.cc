#include <iostream>
#include <vector>
#include "MP_Hungarian.h"

int main() {
    // Create a cost matrix
    std::vector<std::vector<double>> costMatrix_1 = {
        {3, 1, 6},
        {5, 2, 4},
        {7, 3, 8}
    };
    std::vector<std::vector<double>> costMatrix_2 = {
        {3, 1, 6, 2, 8},
        {5, 2, 4, 7, 3},
        {7, 3, 8, 6, 1},
        {2, 5, 3, 0, 9},
        {6, 4, 1, 9, 5}
    };
    std::vector<std::vector<double>> costMatrix_3 = {
        {3, 1, 6},
        {5, 2, 4},
        {7, 3, 8},
        {2, 5, 3},
        {6, 4, 1}
    };

    std::vector<int> assignment;
    double cost = 0;
    // Create an instance of MP_Hungarian
    HungarianAlgorithm mpHungarian;

    // Solve the assignment problem
    cost = mpHungarian.Solve(costMatrix_3,assignment);

    // Print the assignment
    std::cout << "Assignment: ";
    for (int i = 0; i < assignment.size(); i++) {
        std::cout << "(" << i << ", " << assignment[i] << ") ";
    }
    std::cout << "cost: "<< cost;
    std::cout << std::endl;

    return 0;
}