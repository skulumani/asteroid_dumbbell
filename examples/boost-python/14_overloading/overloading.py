#!/usr/bin/env python3

from overload import Example

e = Example()

e.doit()
print(e)
e.doit("Hello")
print(e)

print("---------")
print(e.makeIt("xxx"), e)
print(e.makeIt("abc"), 2), e)
print(e.makeIt("xyz", 3, "abc"), e)
