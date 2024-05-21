import matplotlib.pyplot as plt
import numpy as np

X = np.linspace(0, 100, 10000)

y = [[np.sin(x), np.cos(x)] for x in X]

plt.plot(X, y, label=['sin', 'cos'])

plt.legend()

plt.show()