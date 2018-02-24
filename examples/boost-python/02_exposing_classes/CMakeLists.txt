python_add_module(classes classes.cpp)
file(COPY classes.py DESTINATION .)
add_test(NAME 02_exposing_classes COMMAND ${PYTHON_EXECUTABLE} classes.py)
