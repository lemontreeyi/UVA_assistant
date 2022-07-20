from unittest import result
import numpy as np
import math
station1_x = 0
station1_y = 0.10
station1_z = 0.90
station2_x = 4.85
station2_y = 0.02
station2_z = 1.64
station3_x = 0.04
station3_y = 4.10
station3_z = 1.55
station4_x = 4.90
station4_y = 4.10
station4_z = 1.40
#4.60 5.0 2.67 2.62 
#3.55 4.86 2.21 3.46 
# x = 1.69589564
# y = 2.22384553
# z = 9.14399435
# d1, d2, d3, d4 = np.sqrt((x - station1_x)**2 + (y - station1_y)**2 + (z - station1_z)**2), np.sqrt((x - station2_x)**2 + (y - station2_y)**2 + (z - station2_z)**2), np.sqrt((x - station3_x)**2 + (y - station3_y)**2 + (z - station3_z)**2), np.sqrt((x - station4_x)**2 + (y - station4_y)**2 + (z - station4_z)**2)
# d1, d2, d3, d4 = 3.52492615, 3.74746277, 2.92127930, 3.45175293
D1, D2, D3, D4 = 6.380000, 6.730000, 3.030000, 3.480000
height = 1.50
d1, d2 = math.sqrt(D1**2 - (height - station1_z)**2), math.sqrt(D2**2 - (height - station2_z)**2)
d3, d4 = math.sqrt(D3**2 - (height - station3_z)**2), math.sqrt(D4**2 - (height - station4_z)**2)

print(d1, d2, d3, d4)

b = np.matrix([
    [d1**2 - d4**2 + station4_x**2 + station4_y**2 - station1_x**2 - station1_y**2],
    [d2**2 - d4**2 + station4_x**2 + station4_y**2 - station2_x**2 - station2_y**2],
    [d3**2 - d4**2 + station4_x**2 + station4_y**2 - station3_x**2 - station3_y**2]
])

A = np.matrix([
    [2 * (station4_x - station1_x), 2 * (station4_y - station1_y)], 
    [2 * (station4_x - station2_x), 2 * (station4_y - station2_y)], 
    [2 * (station4_x - station3_x), 2 * (station4_y - station3_y)]
    ])

# u, s, v = np.linalg.svd(A.T * A)
vals, vecs = np.linalg.eig(A.T * A)
print(vals)
print(vecs)
# print(u)
# print(s)
# print(v)
# print(b)
# print(A)
# print(np.linalg.inv(A))
result = np.linalg.inv(A.T * A) * A.T * b
print(result)
