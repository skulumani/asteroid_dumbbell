PYTHON_ADD_MODULE(virtual virtual.cpp)
FILE(COPY virtual.py DESTINATION .)
ADD_TEST(NAME 07_virtual_function COMMAND ${PYTHON_EXECUTABLE} virtual.py)
