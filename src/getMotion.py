import numpy as np

xt = np.zeros((3,1))
xt[0,0],xt[1,0],xt[2,0] = 118, 13, 0.406091

xtt = np.zeros((3,1))
xtt[0,0],xtt[1,0],xtt[2,0] = 113.1661967, 12.31024689, 6.028390862

v = (xtt[0,0] - xt[0,0]) / (5 * np.sin(xt[2,0]) * 0.1)

omega = np.arctan(2 * (xtt[2,0] - xt[2,0]) / (5 * v * 0.1))

print(v, omega)