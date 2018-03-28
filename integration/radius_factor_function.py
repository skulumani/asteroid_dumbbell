
from point_cloud import wavefront

import numpy as np
from scipy import interpolate

import matplotlib.pyplot as plt

def tan_step(x, a, b, delta):
    # out = 1/2 * (np.tanh((x-a)/delta) - np.tanh((x - b)/delta))
    out = 1/2 * ( 1- np.tanh((x-a)/delta))
    return out

def bspline():
    # x = np.arange(0, 2*np.pi+np.pi/4, 2*np.pi/8)
    # y = np.sin(x)
    ctr =np.array([[0, 1], [0.05, 1], [0.1, 1], [0.2, 1], [0.5, 0.2], [0.8, 0.0], [1, 0]])

    x=ctr[:,0]
    y=ctr[:,1]

    #x=np.append(x,x[0])
    #y=np.append(y,y[0])

    tck,u = interpolate.splprep([x,y],k=3, s=0)
    u=np.linspace(-10,10,num=50,endpoint=True)
    out = interpolate.splev(u,tck)
    plt.plot(x, y, 'ro', out[0], out[1], 'b')
    plt.legend(['Points', 'Interpolated B-spline', 'True'],loc='best')


a = 0.5 # left right shift
b = 2 # size of transition
delta = 0.05
x = np.linspace(0, 1, 100)
angle = np.linspace(0, np.pi, 100)
tan_out = tan_step(angle/np.max(angle), a, b, delta)
rad_out = wavefront.radius_scale_factor(angle, a=0.3, delta=0.05)

plt.figure()
plt.plot(x,tan_out, 'g')
plt.plot(angle/np.max(angle), rad_out, 'r' )
plt.title('Tanh interpolation')
# plt.axis([0, 1, 0, 1])
plt.show()
