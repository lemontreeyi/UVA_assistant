import numpy as np

def get_realValue(x, y):
    print("real_x:",(1.007854 * x - 0.09102), " real_y:", (0.980739 * y + 0.072204))

x0, y0 = 4.2262507, 0.7843723
x1, y1 = 0.9094455, 0.6998047
x2, y2 = 0.8587147, 4.1874831
x3, y3 = 4.1896998, 4.1282716
# rx0, ry0 = 0, 0
# rx1, ry1 = 0, 0
# rx2, ry2 = 0, 0
# rx3, ry4 = 0, 0

d21 = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
d43 = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)

hx = d43 / d21
#print("hx:", hx)

d41 = np.sqrt((x3 - x0)**2 + (y3 - y0)**2)
d32 = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

hy = d41 / d32
#print("hy:", hy)

kx = ((4.95 - 1.60) * 2) / (abs(x1 - x0) + abs(x3 - x2))
mean_sumx = ((x1 + x0) + (x2 + x3)) / 2
dx = (4.95 - kx * mean_sumx) / 2

ky = ((4.95 - 1.60) * 2) / (abs(y3 - y0) + abs(y2 - y1))
mean_sumy = ((y3 + y0) + (y2 + y1)) / 2
dy = (4.95 - ky * mean_sumy) / 2

print("kx:", kx)
print("dx:", dx)
print("ky:", ky)
print("dy:", dy)

print(kx * x0 + dx, ky * y0 + dy)
print(kx * x1 + dx, ky * y1 + dy)
print(kx * x2 + dx, ky * y2 + dy)
print(kx * x3 + dx, ky * y3 + dy)

x, y = 4.210564, 0.746827
get_realValue(x,y)