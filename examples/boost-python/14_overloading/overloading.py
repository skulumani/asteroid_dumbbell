#!/usr/bin/env python3

from overload import Example

e = Example()

e.doit()
print(e)
e.doit("Hello")
print(e)

print("---------")
print("{} {}".format(e.makeIt("xxx"), e))
print("{} {}".format(e.makeIt("abc", 2), e))
print("{} {}".format(e.makeIt("xyz", 3, "abc"), e))
