# test out the radius function 

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

x = np.array([0, 0.1, 0.5, 1])
y = np.array([1, 1, 0.5, 0])
domain = np.linspace(0, 1, 100)

k = 2
t = [0, 1, 2, 3, 4, 5, 6]
c = [-1, 2, 0, -1]
spl = interpolate.BSpline(t, c, k)

plt.plot(domain, spl(domain))
plt.show()


