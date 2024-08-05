import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import sosfiltfilt, filtfilt
from filter_model import *


def test_filter(h):
    with open("./Mirae_barometer_0725.csv", "r") as f:
        reader = csv.reader(f)
        data = list(reader)

    data = np.array(data[1:], dtype=float)  # timestamp, pressure, covariance
    timestamp = data[:, 0]
    timestamp = timestamp - timestamp[0]
    pressure = data[:, 1]

    # Apply the filter.
    # filtered = np.convolve(pressure, h, mode="same").clip(
    #     min=1000.994722592397, max=1003.666851310377
    # )
    filtered = filtfilt(h, 1, pressure)

    # Plot the pressure data
    plt.figure()
    plt.plot(timestamp, pressure, label="Raw Pressure")
    plt.plot(timestamp, filtered, label="Filtered Pressure")
    plt.xlabel("Time (s)")
    plt.ylabel("Pressure (hPa)")
    plt.title("FIR filter visualization")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_filter(h_cf03_tb03)
