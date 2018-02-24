#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <string>
#include <sstream>
#include <vector>

// Now we a class and member functions
struct World
{
    std::string mMsg;
    void set(std::string msg)
    {
        mMsg = msg;
    }
    
    void many(boost::python::list msgs)
    {
        long l = len(msgs);
        std::stringstream ss;
        for (long i = 0; i < l; ++i)
        {
            if (i > 0)
            {
                ss << ", ";
            }
            std::string s = boost::python::extract<std::string>(msgs[i]);
            ss << s;
        }
        mMsg = ss.str();
    }
    std::string greet()
    {
        return mMsg;
    }
};


BOOST_PYTHON_MODULE(classes)
{
    using namespace boost::python;

    class_<World>("World")
        .def("greet", &World::greet)
        .def("many", &World::many)
        .def("set", &World::set);
}
