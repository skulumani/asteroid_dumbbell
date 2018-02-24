#!/usr/bin/env python3

import constructor
import numpy as np

c1 = constructor.Constructor("Hello")
print(c1.greet())

c2 = constructor.Constructor(np.pi, 2.6)
print(c2.greet())
