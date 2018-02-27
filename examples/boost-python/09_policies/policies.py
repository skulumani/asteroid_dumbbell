#!/usr/bin/env python

from policies import Example

# e = Example() # constructor is not made available to python
f = Example.factory()
s = Example.singleton()

print("f = {}".format(f))
print("s = {}".format(s))
