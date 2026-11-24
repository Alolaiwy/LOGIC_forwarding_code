"""LOGIC hypervolume-in-range forwarding on one Yanbu traffic run.

Builds a path with the hypervolume-in-range objective f5 for every case in
cases.csv. It sends seven packets along each path and plots PDR against
vehicle count. Only Matplotlib is required.

Usage: python demo.py
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

# Settings from the paper's trace evaluation (Table "trace settings").
CELL_M = 5.0                    # snapshot positions are 5 m grid cells
RANGE_M = 50.0                  # communication range R
MAX_LINKS = 20                  # maximum forwarding links in Yanbu
HORIZON_S = 3                   # prediction horizon H
STEP_S = 1                      # sampling interval Delta t, so K = 3
PAYLOAD_BITS = 1024 * 8         # 1024-byte packets
BANDWIDTH_HZ = 1e6
POWER_W = 1e-3
FREQUENCY_HZ = 5.9e9            # free-space gain, path-loss exponent 2
LIGHT_SPEED = 299792458.0
NOISE_DBM_HZ = -174.0
PACKET_TIMES_S = (0, 3, 6, 9, 12, 15, 18)
TTL_S = 15


def link_rate(distance_m: float) -> float:
    """Shannon rate in bit/s for one link, without interference."""
    gain = (LIGHT_SPEED / (4 * math.pi * FREQUENCY_HZ)) ** 2 / distance_m ** 2
    noise_w = 10 ** (NOISE_DBM_HZ / 10) / 1000 * BANDWIDTH_HZ
    return BANDWIDTH_HZ * math.log2(1 + POWER_W * gain / noise_w)


def in_range(a, b) -> bool:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) * CELL_M ** 2 <= RANGE_M ** 2 + 1e-9


def predicted(cell, motion, tau):
    """Cell coordinates after tau seconds at constant speed and heading."""
    heading, speed = motion
    step = speed / CELL_M * tau
    return (cell[0] + step * math.cos(math.radians(heading)),
            cell[1] + step * math.sin(math.radians(heading)))


def gap(a, b, vehicles, tau) -> float:
    """Predicted distance in cells between the vehicles at cells a and b."""
    if a == b:
        return 0.0
    return math.dist(predicted(a, vehicles[a], tau), predicted(b, vehicles[b], tau))


def normalize(values):
    low, high = min(values), max(values)
    return [0 for _ in values] if high == low else [(v - low) / (high - low) for v in values]


def next_hop(vehicles, sender, destination, visited):
    """Select the relay with the largest hypervolume-in-range score f5."""
    here = math.dist(sender, destination)
    # Candidates in row-major cell order, so ties go to the first cell.
    feasible = [cell for cell in sorted(vehicles)
                if cell not in (sender, destination) and cell not in visited
                and in_range(cell, sender) and math.dist(cell, destination) < here]
    if not feasible:
        return None
    samples = [k * STEP_S for k in range(HORIZON_S // STEP_S + 1)]
    progress, availability = [], []
    for cell in feasible:
        g = a = 0
        for tau in samples:
            s_to_d = gap(sender, destination, vehicles, tau)
            g += (s_to_d - gap(cell, destination, vehicles, tau)) / s_to_d
            a += gap(sender, cell, vehicles, tau) <= RANGE_M / CELL_M
        progress.append(g / len(samples))
        availability.append(a / len(samples))
    scores = [(a + 0.5) * (g + 0.5)
              for a, g in zip(normalize(availability), normalize(progress))]
    return feasible[scores.index(max(scores))]


def build_path(vehicles, source, destination):
    """Local relay choices on the initial snapshot. Returns None if incomplete."""
    path, visited = [source], {source}
    while path[-1] != destination and len(path) <= MAX_LINKS:
        if in_range(path[-1], destination):
            path.append(destination)
            break
        relay = next_hop(vehicles, path[-1], destination, visited)
        if relay is None:
            return None
        path.append(relay)
        visited.add(relay)
    return path if path[-1] == destination else None


def delivered(vehicles, path) -> int:
    """Send seven packets along the fixed path while the vehicles move.

    Each hop takes its own airtime and succeeds only if the link is in range
    when it starts. A packet counts if it arrives within its TTL.
    """
    if path is None:
        return 0
    position = [(r * CELL_M, c * CELL_M) for r, c in path]
    velocity = [(s * math.cos(math.radians(h)), s * math.sin(math.radians(h)))
                for h, s in (vehicles[cell] for cell in path)]
    count = 0
    for sent in PACKET_TIMES_S:
        t = float(sent)
        for i in range(len(path) - 1):
            distance = math.hypot(
                position[i][0] - position[i + 1][0] + (velocity[i][0] - velocity[i + 1][0]) * t,
                position[i][1] - position[i + 1][1] + (velocity[i][1] - velocity[i + 1][1]) * t)
            if distance > RANGE_M:
                break
            if distance > 0:
                t += PAYLOAD_BITS / link_rate(distance)
        else:
            count += t - sent <= TTL_S
    return count


def load(folder: Path):
    snapshots = defaultdict(dict)
    with open(folder / "vehicles.csv", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            cell = (int(row["row"]), int(row["col"]))
            snapshots[row["snapshot"]][cell] = (float(row["heading_deg"]), float(row["speed_mps"]))
    with open(folder / "cases.csv", newline="", encoding="utf-8") as stream:
        cases = [(row["snapshot"], int(row["vehicles"]),
                  (int(row["source_row"]), int(row["source_col"])),
                  (int(row["destination_row"]), int(row["destination_col"])))
                 for row in csv.DictReader(stream)]
    return snapshots, cases


def evaluate(snapshots, cases):
    """PDR in percent per vehicle count. Failed routes stay in the denominator."""
    sent, received = defaultdict(int), defaultdict(int)
    for key, count, source, destination in cases:
        vehicles = snapshots[key]
        received[count] += delivered(vehicles, build_path(vehicles, source, destination))
        sent[count] += len(PACKET_TIMES_S)
    return {count: 100 * received[count] / sent[count] for count in sorted(sent)}


def plot(pdr, output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    counts = list(pdr)
    fig, axis = plt.subplots(figsize=(6, 4), layout="constrained")
    axis.plot(counts, list(pdr.values()), color="#1945ad", marker="o", linewidth=2)
    axis.set(xlabel="Number of vehicles", ylabel="Packet delivery ratio (%)",
             xticks=counts, ylim=(0, 5 * math.ceil(max(pdr.values()) / 5) + 5),
             title="Hypervolume-in-range greedy, Yanbu, traffic run 101")
    axis.grid(axis="y", color="#d9dfe5", linewidth=0.7)
    axis.spines[["top", "right"]].set_visible(False)
    fig.savefig(output, dpi=200, facecolor="white")
    plt.close(fig)


def main() -> None:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, default=here, help="folder with the two CSV files")
    parser.add_argument("--output", type=Path, default=here / "demo_results.png")
    args = parser.parse_args()
    snapshots, cases = load(args.data)
    pdr = evaluate(snapshots, cases)
    print("Vehicles   PDR (%)")
    for count, value in pdr.items():
        print(f"{count:8d}   {value:7.2f}")
    plot(pdr, args.output)
    print(f"Saved {args.output.resolve()}")


if __name__ == "__main__":
    main()
