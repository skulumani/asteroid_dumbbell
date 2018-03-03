PYTHON_ADD_MODULE(myexceptions myexceptions.cpp)
file(COPY myexceptions.py DESTINATION .)
add_test(NAME 12_exceptions COMMAND ${PYTHON_EXECUTABLE} myexceptions.py)
