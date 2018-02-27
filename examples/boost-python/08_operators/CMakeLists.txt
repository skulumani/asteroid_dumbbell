PYTHON_ADD_MODULE(operators operators.cpp)
FILE(COPY operators.py DESTINATION .)
ADD_TEST(NAME 08_operators COMMAND ${PYTHON_EXECUTABLE} operators.py)
