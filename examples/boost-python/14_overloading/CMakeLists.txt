
PYTHON_ADD_MODULE(overload overloading.cpp)
FILE(COPY overloading.py DESTINATION .)
ADD_TEST(NAME 14_overloading COMMAND ${PYTHON_EXECUTABLE} overloading.py)

