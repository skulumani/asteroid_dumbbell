#include <sstream>
#include <string>
#include <vector>

#include <boost/python.hpp>

class Example {
    public:
        Example() {}
        void add(const std::string& s) {
            mS.push_back(s);
        }

        std::vector<std::string>::iterator begin() {
            return mS.begin();
        }

        std::vector<std::string>::iterator end() {
            return mS.end();
        }

    private:
        std::vector<std::string> mS;
};

BOOST_PYTHON_MODULE(iterators) {
    boost::python::class_<Example>("Example")
        .def("strings", boost::python::range(&Example::begin, &Example::end))
        .def("add", &Example::add)
        ;
}
