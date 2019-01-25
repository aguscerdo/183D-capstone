import numpy as np
import matplotlib.pyplot as plt
import csv


def plotter(path):
    with open(path, 'r') as fp:
        reader = csv.reader(fp)
        dp = np.asarray(list(reader), dtype=float)

    x = dp[:, 0]
    y = dp[:, 1]
    z = dp[:, 2]

    plt.scatter(x, y, z)
    plt.axes()
    plt.xlabel('X Mag')
    plt.ylabel('Y Mag')
    plt.show()

    xmin = np.min(x)
    xmax = np.max(x)
    ymin = np.min(y)
    ymax = np.max(y)

    x_off = (xmax + xmin) / 2
    y_off = (ymax + ymin) / 2

    x_scale = (xmax - xmin) / 2
    y_scale = (ymax - ymin) / 2

    print('x_bias: {}, x_scale; y_bias: {}, y_scale: {}'.format(x_off, x_scale, y_off, y_scale))


if __name__ == '__main__':
    f = 'mag.csv'
    plotter(f)