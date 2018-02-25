# this will create a so file for import into python
PYTHON_ADD_MODULE(test_numpy test_numpy.cpp)
file(COPY test_numpy.py DESTINATION .)
add_test(NAME 04_numpy_example COMMAND ${PYTHON_EXECUTABLE} test_numpy.py)

