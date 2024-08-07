import csv
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from filter_model import *


def test_filter(h, config, raw=False):
    with open("./Mirae_barometer_0725.csv", "r") as f:
        reader = csv.reader(f)
        data = list(reader)

    h = np.array(h)
    data = np.array(data[1:], dtype=float)  # timestamp, pressure, covariance
    timestamp = data[:, 0]
    timestamp = timestamp - timestamp[0]
    pressure = data[:, 1]

    N = len(h)
    buffer = deque([], maxlen=N)

    start_time = time.time()
    filtered_list = []
    # Apply the filter.
    for t, p in zip(timestamp, pressure):
        buffer.append(p)

        if len(buffer) != N:
            filtered_list.append(p)
            continue

        filtered_list.append(np.convolve(h, buffer, mode="valid"))
    print(f"Elapsed time{config}: ", time.time() - start_time)

    # Plot the pressure data
    if raw:
        plt.plot(timestamp, pressure, label="Raw Pressure")
    plt.plot(timestamp, filtered_list, label=rf"Filtered Pressure ${config}$")
    plt.xlabel("Time (s)")
    plt.ylabel("Pressure (hPa)")
    plt.title("FIR filter visualization")
    plt.legend()
    plt.tight_layout()


if __name__ == "__main__":
    plt.figure()
    test_filter(h_cf03_tb03, config=f"f_L: {0.3}Hz, b_L: {0.3}Hz", raw=True)
    test_filter(h_cf3_tb06, config=f"f_L: {3}Hz, b_L: {0.6}Hz")
    test_filter(h_cf6_tb06, config=f"f_L: {6}Hz, b_L: {0.6}Hz")
    test_filter(h_cf03_tb3, config=f"f_L: {0.3}Hz, b_L: {3}Hz")
    plt.show()
