python_add_module(constructor constructor.cpp)
file(COPY constructor.py DESTINATION .)
add_test(NAME 03_constructor COMMAND ${PYTHON_EXECUTABLE} constructor.py)
