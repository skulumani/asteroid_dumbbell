python_add_module(iterators iterators.cpp)
file(COPY iterators.py DESTINATION .)
add_test(NAME 11-iterators COMMAND ${PYTHON_EXECUTABLE} iterators.py)
