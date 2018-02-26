#!/usr/bin/env python3

import member

m1 = member.SomeClass('Hello Dude')
print("Name = {}".format(m1.name))

m1.name = 'Shankar'
print("Name = {}".format(m1.name))

print("Num = {}".format(m1.num))
m1.num = 10
print("Num = {}".format(m1.num))

m1.value = 150
print("Value = {}".format(m1.value))
