PYTHON_ADD_MODULE(policies policies.cpp)
FILE(COPY policies.py DESTINATION .)
ADD_TEST(NAME 09_policies COMMAND ${PYTHON_EXECUTABLE} policies.py)
