#include <sstream>
#include <string>

// This allows us to expose overloaded operators to Python
class NumberLike {
    public:
        NumberLike(int n = 0): mN(n) {}
        NumberLike& operator += (int i) {
            mN += i;
            return *this;
        }
        // These operators define our printing will work in python
        std::string str() const {
            std::stringstream s;
            s << mN;
            return s.str();
        }
        std::string repr() const {
            std::stringstream s;
            s << "NumberLike(" << mN << ")";
            return s.str();
        }
    private:
        int mN;
};

// This will call the overloaded += operator to add i to the private member attribute
NumberLike operator+(NumberLike n, int i) {
    n += i;
    return n;
}

#include <boost/python.hpp>
namespace bp = boost::python;

BOOST_PYTHON_MODULE(operators) {
    bp::class_<NumberLike>("NumberLike")
        .def(bp::init< bp::optional<int> >())
        .def(bp::self + int())
        .def("__str__", &NumberLike::str)
        .def("__repr__", &NumberLike::repr)
    ;
}


