#!/usr/bin/env python3

from operators import NumberLike

n = NumberLike(7)
m  = n + 2 # create new NumberLike instance and apply + operator 
print(m)

n0 = NumberLike()
m0 = n0 + 1
print(m0)

print(repr(m0)) # call repr method

