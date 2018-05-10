#include <iostream>
#include <vector>

int main ( void ) {
    std::vector<int> vec(10);
    #pragma omp parallel for
    for (int ii = 0; ii < 10; ++ii) {
        vec[ii] = ii;
    }

    // now print to screen
    for (std::vector<int>::size_type ii = 0; ii < vec.size(); ++ii) {
        std::cout << vec[ii] << " ";
    }
    std::cout << std::endl;

}
