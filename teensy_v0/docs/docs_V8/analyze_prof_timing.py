import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LOG_DIR = Path(
    "/Users/k_pochkaev/github/flight_computers/teensy_v0/flight_logs"
)
LOG_FILE = LOG_DIR / "20251206_195224_flight42.log"  # adjust if needed

PROF_RE = re.compile(
    r"PROF,loop_us=(\d+),sens=(\d+),radio=(\d+),pwr=(\d+),log=(\d+),ui=(\d+)"
)


def load_prof_samples(path: Path):
    samples = []
    with path.open() as f:
        for line in f:
            m = PROF_RE.search(line)
            if not m:
                continue
            loop_us, sens, radio, pwr, log, ui = map(int, m.groups())
            samples.append(
                dict(
                    loop_us=loop_us,
                    sens=sens,
                    radio=radio,
                    pwr=pwr,
                    log=log,
                    ui=ui,
                )
            )
    return samples


def summarize(samples):
    if not samples:
        print("No PROF samples found.")
        return
    keys = ["loop_us", "sens", "radio", "pwr", "log", "ui"]
    print(f"Samples: {len(samples)}")
    for k in keys:
        arr = np.array([s[k] for s in samples], dtype=float)
        print(
            f"{k:8s}: mean={arr.mean():8.1f} us, "
            f"p50={np.percentile(arr, 50):7.1f}, "
            f"p90={np.percentile(arr, 90):7.1f}, "
            f"max={arr.max():8.1f}"
        )


def plot_timeseries(samples):
    if not samples:
        return
    keys = ["sens", "radio", "pwr", "log", "ui"]
    x = np.arange(len(samples))
    fig, ax = plt.subplots(figsize=(10, 5))
    for k in keys:
        y = [s[k] for s in samples]
        ax.plot(x, y, label=k)
    ax.set_xlabel("Sample index")
    ax.set_ylabel("Time (us)")
    ax.set_yscale("log")
    ax.grid(True, which="both", linestyle="--", alpha=0.3)
    ax.legend()
    ax.set_title(LOG_FILE.name)
    plt.tight_layout()
    plt.show()


def main():
    samples = load_prof_samples(LOG_FILE)
    summarize(samples)
    plot_timeseries(samples)


if __name__ == "__main__":
    main()

