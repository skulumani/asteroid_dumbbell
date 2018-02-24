#include <string>
#include <sstream>
#include <boost/python.hpp>

struct Constructor
{
    Constructor(std::string msg) : mMsg(msg) {}
    Constructor(double x, double y)
    {
        std::stringstream os;
        os << x << ":" << y << std::ends;
        set(os.str());
    }
    void set(std::string msg)
    {
        mMsg = msg;
    }
    std::string greet()
    {
        return mMsg;
    }
    std::string mMsg;
};

BOOST_PYTHON_MODULE(constructor)
{
    boost::python::class_<Constructor>("Constructor", boost::python::init<std::string>())
        .def(boost::python::init<double, double>())
        .def("greet", &Constructor::greet)
        .def("set", &Constructor::set);
}
