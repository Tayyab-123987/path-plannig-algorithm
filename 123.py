import matplotlib.pyplot as plt
import numpy as np
x_values = [2,0.4,6,5,2]
y_values = [1,6.2,5,1,1]
n=15
dx=0.5
dy=dx

x=(dx/2.)+dx*np.arange(0,n).reshape((n,1))
y=(dy/2.)+dy*np.arange(0,n).reshape((1,n))
x
xcoords=np.kron(np.ones_like(x.T),x)
ycoords=np.kron(y,np.ones_like(y.T))
plt.plot(xcoords,ycoords)
plt.plot(ycoords,xcoords)
plt.plot(x_values,y_values)
plt.show()

