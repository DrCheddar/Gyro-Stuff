import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import sys
if sys.version_info[0] < 3:
    from StringIO import StringIO
else:
    from io import StringIO

SAVE = True
SHOW = False

def graph_data(csv_data, title = "PID Tuner", x = "time", y1 = "setpoint", y2 = "actual", x_label = "Time in ms", y_label = "Position"):
    df = pd.read_csv(csv_data)
    x_values = df[x].values.tolist()
    ax = df.plot.line(x = x, y = y1)
    plot = df.plot.line(ax = ax, x = x, y = y2, grid = True, title = title, xticks = x_values)
    plot.set_xticklabels(x_values)
    return plot

def save_or_show(plot, png_filename, save = SHOW):
    if save == SAVE:
        figure = plot.get_figure()
        figure.savefig(png_filename)
    else:
        plt.show()

def main(csv_path):
    csv_data = open(csv_path, 'r')
    plot = graph_data(csv_data)
    save_or_show(plot, "pid.png", SHOW)

if __name__ == "__main__":
    main("test_data.csv")
