# this will create a so file for import into python
add_library(inheritance SHARED inheritance.cpp)
set_target_properties(inheritance PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(inheritance PROPERTIES DEFINE_SYMBOL "BOOST_ALL_NO_LIB")
target_link_libraries(inheritance ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

file(COPY inheritance.py DESTINATION .)
add_test(NAME 06_inheritance COMMAND ${PYTHON_EXECUTABLE} inheritance.py)

