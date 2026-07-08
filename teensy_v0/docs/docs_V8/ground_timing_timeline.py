import matplotlib.pyplot as plt


def plot_example_timeline():
    """
    Example 1-second timeline for the Ground Station.
    Adjust event timings/durations to match measurements from hardware.
    """
    # Events are approximate; tune based on your profiling.
    # This version reflects current cadences:
    # - RS485 TX: ~every 150 ms
    # - UI update: ~every 250 ms
    # - GPS read: ~every 1000 ms
    # - Sensors (BMP180): ~every 2000 ms (may show at most once in this 1 s window)
    events = {
        "RS485_TX": [
            (0, 3),
            (150, 3),
            (300, 3),
            (450, 3),
            (600, 3),
            (750, 3),
            (900, 3),
        ],
        "UI_Update": [
            (0, 2),
            (250, 2),
            (500, 2),
            (750, 2),
        ],
        # Ground GPS: processed about once per second
        "GPS_Read": [
            (10, 1),
        ],
        # LoRa updates modeled as several short bursts when packets arrive
        "LoRa_Update": [
            (50, 2),
            (250, 2),
            (450, 2),
            (650, 2),
            (850, 2),
        ],
        # Example: logging bursts every 250 ms (flight / recovery phases)
        "Logging": [(t, 3) for t in range(200, 1000, 250)],
        # Sensors (BMP180) read roughly every 2000 ms; at most one in this window
        "Sensors": [
            (20, 0.8),
        ],
    }


    colors = {
        "RS485_TX": "tab:red",
        "UI_Update": "tab:blue",
        "GPS_Read": "tab:green",
        "LoRa_Update": "tab:olive",
        "Logging": "tab:orange",
        "Sensors": "tab:purple",
    }

    fig, ax = plt.subplots(figsize=(10, 4))

    yticks = []
    yticklabels = []
    y = 10
    height = 8

    for name, evts in events.items():
        bars = [(start, dur) for (start, dur) in evts]
        ax.broken_barh(bars, (y, height), facecolors=colors.get(name, "gray"))
        yticks.append(y + height / 2)
        yticklabels.append(name)
        y += height + 5

    ax.set_ylim(0, y + 5)
    ax.set_xlim(0, 1000)
    ax.set_xlabel("Time (ms)")
    ax.set_yticks(yticks)
    ax.set_yticklabels(ytlabels := yticklabels)
    ax.grid(True, axis="x", linestyle="--", alpha=0.4)
    ax.set_title("Ground Station – Example 1s Timeline")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_example_timeline()
