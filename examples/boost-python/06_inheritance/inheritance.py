#!/usr/bin/env python3

import inheritance

b = inheritance.Base()
d = inheritance.Derived()

inheritance.fb(b)
inheritance.fb(d)

# this will fail fd is only for Derived objects
# inheritance.fd(b)
inheritance.fd(d)

# instantiate a new object
x = inheritance.factory()
inheritance.fb(x)
