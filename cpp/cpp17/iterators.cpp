#include <stdio.h>
#include <iterator>
#include <iostream>
#include <forward_list>
#include <list>
#include <vector>


//This example demonstrates how different categories of iterators work in the C++17 and which containers provide them.


int main(){

    // input_iterator_tag
    std::istream_iterator<int> it(std::cin), end;
    while (it != end)
        std::cout << *it++ << "\n";

    // forward_iterator_tag

    std::forward_list<int> fl = {1,2,3};
    for (auto i = fl.begin(); i != fl.end(); ++i) {
        std::cout << *i << " ";
    }

    // bidirectional_iterator_tag
    std::list<int> lst = {1,2,3};
    auto bd = lst.end();
    while (bd != lst.begin()) {
        --bd;
        std::cout << *bd << " ";
    }

    //random_access_iterator_tag
    std::vector<int> v = {10, 20, 30};
    std::cout << v[1] << "\n";    // 20
    auto ra = v.begin() + 2;      



    return 0;
}