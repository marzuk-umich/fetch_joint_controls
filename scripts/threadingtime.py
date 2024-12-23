import numpy as np

a = np.array([1, 2, 3])
b = np.tile(a, 3)
print(a.reshape(4,))  # Output: [1 2 3 1 2 3 1 2 3]
