#!/usr/bin/env python3

from sys import stdout
from iterators import Example

e = Example()

for letter in ("a", "b", "c"):
    e.add(letter)

print([ s for s in e.strings() ])
