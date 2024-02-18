import numpy as np

m0 = [
    [1, 2, 1, 1],
    [4, 5, 6, 1],
    [2, 3, 5, 1],
    [2, 3, 5, 1]
]

m1 = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

i = 0
j = 0
k = 0

m01 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]


for i in range(0, 4):

    for k in range(0, 4):

        for j in range(0, 4):

            m01[i][k] = m01[i][k] + m0[i][j] * m1[j][k]

            j = j + 1

        k = k + 1

    i = i + 1

print(np.matrix(m01))