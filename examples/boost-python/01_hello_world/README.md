The Boost Python Library is a framework for interfacing Python and C++.
It doesn't require any special tools or require changes to the C++ code, making it ideal for 3rd party libraries that don't exist in Python.

This directory has a single C++ function.

A Boost wrapper is used to expose it to Python.

The C++ source code is built as a shared library which Python can import as usual.
