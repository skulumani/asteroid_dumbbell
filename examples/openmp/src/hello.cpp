#include <iostream>
#include <omp.h>

int main() {
    #pragma omp parallel
	{
        int ID = omp_get_thread_num();
		std::cout << "Hello " << ID << std::endl;
		std::cout << "Hello again " << ID << std::endl;
    }
    return 0;
}
