## Finding memory leaks using Valgrind

[Valgrind](valgrind.org) is a free software tool designed to profile your code.
It works for any programming language, but is often used for C/C++ since that's where many bugs exist.

First you need to install Valgrind

~~~
sudo apt-get install valgrind
~~~

Then make sure your compiled code included debugging symbols

~~~
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4
~~~

Run Valgrind on your program

~~~
valgrind --leak-check=yes myprog arg1 arg2
~~~


