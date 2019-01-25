import numpy as np
import matplotlib.pyplot as plt
import csv


def plotter(path):
    with open(path, 'r') as fp:
        reader = csv.reader(fp)
        dp = np.asarray(list(reader))

    x = dp[:, 0]
    y = dp[:, 1]
    z = dp[:, 2]

    plt.plot(x, y)
    plt.xlabel('X Mag')
    plt.ylabel('Y Mag')
    plt.show()