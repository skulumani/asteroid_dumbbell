# this will create a so file for import into python
add_library(member SHARED member.cpp)
set_target_properties(member PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(member PROPERTIES DEFINE_SYMBOL "BOOST_ALL_NO_LIB")
target_link_libraries(member ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

file(COPY member.py DESTINATION .)
add_test(NAME 05_class_members COMMAND ${PYTHON_EXECUTABLE} member.py)

