#!/usr/bin/env python3

import classes

t = classes.World()
t.set('hello world')
print(t.greet())

t.many(['Hello', 'World', 'And', 'Another', 'String', 'Instide', 'A List'])
print(t.greet())
