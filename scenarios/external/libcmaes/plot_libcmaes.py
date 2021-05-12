
from numpy import genfromtxt
import sys
import numpy as np
import matplotlib.pyplot as plt

fileName = sys.argv[1]

my_data = genfromtxt(fileName, delimiter=',')

print my_data

data = np.array(my_data)
x = data[:,0]
y = data[:,1]


print x
print y

fig, (training_ax) = plt.subplots(1, 1, sharey=False, sharex=True)
training_ax.scatter(x, y, linewidth=2.0)
# error_std = training_ax.fill_between(range(len(mean)), mean-stddev, mean+stddev, facecolor='blue', alpha=0.5)
training_ax.set_title('Training Progress')
training_ax.set_ylabel("X Coordinate")
training_ax.set_xlabel("Y coordinate")

plt.show()