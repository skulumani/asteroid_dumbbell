#include <iostream>

int main ( void ) {
    std::cout << "Begining of loop" << std::endl;

    #pragma omp parallel for
    for (int ii = 0; ii < 10; ++ ii) {
        std::cout << ii;
    }
    std::cout << std::endl << "End of loop" << std::endl;
    return 0;
}
