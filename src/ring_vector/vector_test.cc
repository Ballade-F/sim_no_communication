#include <iostream>
#include "ring_vector.hpp"

int main() {
    // Create a ring vector of integers
    RING_VECTOR<int> ringVector;

    // Add elements to the ring vector
    ringVector.push_back(10);
    ringVector.push_back(20);
    ringVector.push_back(30);
    ringVector.push_back(40);
    ringVector.push_back(50);

    // Print the elements in the ring vector
    for(uint8_t i = 0; i < ringVector.size(); i++) 
    {
        std::cout << ringVector.at(i) << " ";
    }
    std::cout << std::endl;

    int test = 60;
    for(uint8_t i = 0; i < 10; i++) 
    {
        ringVector.push_pop(test);
        for(uint8_t j = 0; j < ringVector.size(); j++) 
        {
            std::cout << ringVector.at(j) << " ";
        }
        std::cout << std::endl;
        test += 10;
    }
    std::cout << ringVector.back() << " ";
    std::cout << ringVector.size() << " ";
    std::cout << ringVector.at(ringVector.size()-2) << " ";


    return 0;
}