#include <stdio.h>
#include <memory>
#include <random>
#include <algorithm>
#include <vector>

//This example demonstrates things were removed from the C++17.


int main(){
    
    // Before:
    //register int a = 0; 
    // After:
    int a = 0; //keyword register was removed. reason - useless
    
    // Before:
    //bool flag = true; flag ++; 
    // After:
    bool flag = true;
    flag = !flag; // boolean incremeting was removed. unsafe

    // Before:
    // void foo() throw(std::runtime_error);
    // After:
    void foo(); // dynamic exception specifications were removed. rarely useful

    // Before:
    // std::auto_ptr<int> p(new int(42)); 
    // After:
    auto p = std::make_unique<int>(42); // safe way, modern ownership 

    // Before:
    //std::random_shuffle(v.begin(), v.end());
    // After:
    std::vector<int> v = {1, 2, 3, 4, 5};
    std::mt19937 gen(std::random_device{}());
    std::shuffle(v.begin(), v.end(), gen);    // no more rand() function as RNG only, now you should choose 

    return 0;
}