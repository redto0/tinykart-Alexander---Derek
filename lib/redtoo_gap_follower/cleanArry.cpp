// functions.cpp
#include <iostream>
#include "cleanArry.hpp"
#include <ld06.hpp>
#include <vector>


auto cleanArry(const std::vector<ScanPoint> &scan) {
    
    int counter = 0;
    for(auto scanner : scan){
        /* code */
        
        if(scanner.dist( ScanPoint::zero() ) == 0) {
            counter++;
        }
    }

    std::vector<ScanPoint> cleanArry[counter];

    int i = 0;
    for(auto scanner : scan){
        if(scanner.dist( ScanPoint::zero() ) == 0) {
            cleanArr[i] = Scanner;
            i++;
        }
    }

    return 0;
    
}

