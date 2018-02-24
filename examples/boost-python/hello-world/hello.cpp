#include <boost/python.hpp>

// A function
char const* greet()
{
    return "hello world!";
}

// Now we a class and member functions
struct World
{
    void set(std::string msg)
    {
        this->msg = msg;
    }
    std::string greet()
    {
        return msg;
    }
    std::string msg;
};

BOOST_PYTHON_MODULE(hello)
{
    using namespace boost::python;
    def("greet", greet);

    class_<World>("World")
        .def("greet", &World::greet)
        .def("set", &World::set);
}
