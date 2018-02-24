
#include <iostream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/enum.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Vector_2 Vector_2;

int main()
{
    Point_2 p(1.0, 1.0), q;
    Vector_2 v;
    v = p - CGAL::ORIGIN;
    q = CGAL::ORIGIN + v;

    std::cout << "Point p: " << p << std::endl;
    std::cout << "Vector v = p - ORIGIN = " << v << std::endl;
    std::cout << "Point q = v + ORIGIN = " << q << std::endl;
    assert(p == q);
    return 0;
}
