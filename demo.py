"""Run four LOGIC objectives on a small Yanbu trace and save one 2 x 2 figure.

Usage: python demo.py
Only NumPy and Matplotlib are required. No workspace files are imported.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


# All distances are metres, speeds m/s, durations seconds and payloads bits.
RANGE_M = 50.0
PAYLOAD_BITS = 1024 * 8
BANDWIDTH_HZ = 1_000_000.0
POWER_W = 0.001
NOISE_W_PER_HZ = 1e-10
LOSS_EXPONENT = 2.0
HORIZON_S = 3.0
PREDICTION_STEP_S = 0.25
SOFT_WEIGHTS = (0.5, 0.5)
OBSERVATION_S = 20.0
TTL_S = 15.0
PACKET_TIMES_S = np.arange(0.0, OBSERVATION_S, 3.0)
MAX_LINKS = 20
COUNTS = tuple(range(30, 101, 10))
SEED = 20261020
OBJECTIVES = (
    ("f1", "Greedy", "#009dad", "D"),
    ("f2", "Predicted greedy", "#1945ad", "v"),
    ("f3", "Hard-in-range greedy", "#d64035", "o"),
    ("f4", "Soft-in-range greedy", "#268b45", "^"),
)


def transmission_seconds(distance_m: float) -> float:
    """Unit-reference-gain Shannon model, without interference or MAC overhead."""
    gain = max(distance_m, 1e-12) ** (-LOSS_EXPONENT)
    rate_bps = BANDWIDTH_HZ * np.log2(
        1.0 + POWER_W * gain / (NOISE_W_PER_HZ * BANDWIDTH_HZ)
    )
    return float(PAYLOAD_BITS / rate_bps)


def normalize(values: np.ndarray) -> np.ndarray:
    """Min-max normalization over the current feasible candidates."""
    span = float(np.ptp(values))
    return (values - values.min()) / span if span > 0 else np.zeros_like(values)


def objective_scores(
    positions: np.ndarray,
    velocities: np.ndarray,
    sender: int,
    destination: int,
    candidates: np.ndarray,
    objective: str,
) -> np.ndarray:
    """The four scores f1--f4, with candidate-specific prediction horizons."""
    if objective == "f1":
        return -np.linalg.norm(positions[candidates] - positions[destination], axis=1)
    if objective not in {"f2", "f3", "f4"}:
        raise ValueError(f"Unknown objective: {objective}")

    progress, retention, available = [], [], []
    for candidate in candidates:
        distance = float(np.linalg.norm(positions[sender] - positions[candidate]))
        horizon = min(HORIZON_S, transmission_seconds(distance))
        samples = np.arange(int(np.floor(horizon / PREDICTION_STEP_S)) + 1)
        times = samples[:, None] * PREDICTION_STEP_S
        source = positions[sender] + times * velocities[sender]
        relay = positions[candidate] + times * velocities[candidate]
        target = positions[destination] + times * velocities[destination]
        source_to_target = np.linalg.norm(source - target, axis=1)
        relay_to_target = np.linalg.norm(relay - target, axis=1)
        separation = np.linalg.norm(source - relay, axis=1)
        progress.append(np.mean(
            (source_to_target - relay_to_target) / np.maximum(source_to_target, 1e-12)
        ))
        retention.append(np.mean((distance - separation) / max(distance, 1e-12)))
        available.append(bool(np.all(separation < RANGE_M)))

    progress = np.asarray(progress)
    if objective == "f2":
        return progress
    if objective == "f3":
        return np.where(available, progress, -np.inf)
    return (SOFT_WEIGHTS[0] * normalize(np.asarray(retention))
            + SOFT_WEIGHTS[1] * normalize(progress))


def construct_route(
    positions: np.ndarray,
    velocities: np.ndarray,
    source: int,
    destination: int,
    objective: str,
) -> list[int] | None:
    """Make local relay choices on the initial snapshot; reject incomplete paths."""
    path = [source]
    visited = {source}
    to_destination = np.linalg.norm(positions - positions[destination], axis=1)
    for _ in range(MAX_LINKS):
        sender = path[-1]
        distances = np.linalg.norm(positions - positions[sender], axis=1)
        # Direct delivery takes priority when the destination is already in range.
        if distances[destination] <= RANGE_M:
            return path + [destination]
        feasible = ((distances <= RANGE_M)
                    & (to_destination < to_destination[sender]))
        feasible[list(visited)] = False
        candidates = np.flatnonzero(feasible)
        if not candidates.size:
            return None
        scores = objective_scores(
            positions, velocities, sender, destination, candidates, objective
        )
        if not np.any(np.isfinite(scores)):
            return None  # Includes the case where every f3 score is -infinity.
        next_hop = int(candidates[np.argmax(scores)])
        path.append(next_hop)
        visited.add(next_hop)
        if next_hop == destination:
            return path
    return None


def delivered_packets(
    positions: np.ndarray, velocities: np.ndarray, path: list[int] | None
) -> int:
    """Evaluate seven independent packets on the fixed path using moving nodes.

    Each hop checks range at transmission start. There is no retransmission,
    route repair, queueing or interference. Only arrival at the intended final
    destination before both deadlines counts as delivery.
    """
    if path is None:
        return 0
    delivered = 0
    for generated in PACKET_TIMES_S:
        now = float(generated)
        deadline = min(OBSERVATION_S, now + TTL_S)
        for sender, receiver in zip(path, path[1:]):
            separation = (positions[sender] + velocities[sender] * now
                          - positions[receiver] - velocities[receiver] * now)
            distance = float(np.linalg.norm(separation))
            if distance > RANGE_M:
                break
            now += transmission_seconds(distance)
            if now > deadline:
                break
        else:
            delivered += 1
    return delivered


def load_sample(path: Path) -> list[dict]:
    """Read the bundled trace subset and validate its essential input fields."""
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or not data.get("snapshots"):
        raise ValueError("Expected a version-1 trace sample containing snapshots.")
    snapshots = []
    for frame in data["snapshots"]:
        vehicles = sorted(frame["vehicles"], key=lambda vehicle: vehicle["id"])
        ids = [vehicle["id"] for vehicle in vehicles]
        positions = np.array([[v["x_m"], v["y_m"]] for v in vehicles], dtype=float)
        speed = np.array([v["speed_mps"] for v in vehicles], dtype=float)
        angle = np.array([v["sumo_heading_deg"] for v in vehicles], dtype=float)
        # SUMO uses degrees clockwise from north. Convert to Cartesian x/y.
        radians = np.deg2rad(90.0 - angle)
        velocities = speed[:, None] * np.column_stack((np.cos(radians), np.sin(radians)))
        pairs = [(ids.index(p["source"]), ids.index(p["destination"]))
                 for p in frame["endpoint_pairs"]]
        if (len(ids) < max(COUNTS) or len(set(ids)) != len(ids)
                or not np.isfinite(positions).all() or not np.isfinite(velocities).all()
                or np.any(speed < 0) or np.any((angle < 0) | (angle > 360))
                or not pairs or any(source == target for source, target in pairs)):
            raise ValueError(f"Invalid vehicles or endpoints at t={frame['time_s']}.")
        snapshots.append(dict(time_s=frame["time_s"], positions=positions,
                              velocities=velocities, pairs=pairs))
    return snapshots


def evaluate(snapshots: list[dict]) -> dict[str, np.ndarray]:
    """Vary nested populations with the same pinned endpoints for all objectives."""
    totals = {name: np.zeros(len(COUNTS), dtype=int) for name, *_ in OBJECTIVES}
    cases = 0
    for frame in snapshots:
        positions, velocities = frame["positions"], frame["velocities"]
        pinned = sorted({i for pair in frame["pairs"] for i in pair})
        if len(pinned) > min(COUNTS):
            raise ValueError("The endpoint set exceeds the lowest vehicle count.")
        others = [i for i in range(len(positions)) if i not in pinned]
        rng = np.random.default_rng(np.random.SeedSequence([SEED, int(frame["time_s"])]))
        ranking = rng.permutation(others)
        for column, count in enumerate(COUNTS):
            selected = np.sort(np.r_[pinned, ranking[:count - len(pinned)]]).astype(int)
            local_index = {int(original): local for local, original in enumerate(selected)}
            xy, velocity = positions[selected], velocities[selected]
            for source, destination in frame["pairs"]:
                for name, *_ in OBJECTIVES:
                    route = construct_route(xy, velocity, local_index[source],
                                            local_index[destination], name)
                    totals[name][column] += delivered_packets(xy, velocity, route)
        cases += len(frame["pairs"])
    # Failed routes remain in the offered-packet denominator.
    offered = cases * len(PACKET_TIMES_S)
    return {name: 100.0 * values / offered for name, values in totals.items()}


def plot_results(results: dict[str, np.ndarray], snapshots: list[dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True,
                             layout="constrained")
    ceiling = max(5.0, 5.0 * np.ceil(max(float(v.max()) for v in results.values()) / 5.0))
    for index, (axis, (name, title, color, marker)) in enumerate(zip(axes.flat, OBJECTIVES)):
        axis.plot(COUNTS, results[name], color=color, marker=marker, linewidth=2,
                  markersize=6)
        axis.set_title(f"({chr(97 + index)}) {title}  $f_{index + 1}$", fontsize=12)
        axis.set(xlim=(28, 102), ylim=(0, ceiling * 1.08), xticks=COUNTS)
        axis.grid(axis="y", color="#d9dfe5", linewidth=0.7)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(labelbottom=True, labelleft=True)
        axis.set_xlabel("Number of vehicles")
        axis.set_ylabel("Packet delivery ratio (%)")
    pairs = sum(len(frame["pairs"]) for frame in snapshots)
    fig.suptitle("Yanbu trace forwarding demonstration\n"
                 f"One traffic run, {len(snapshots)} snapshots, {pairs} matched endpoint pairs",
                 fontsize=14)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200, facecolor="white")
    plt.close(fig)


def main() -> None:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, default=here / "trace_sample.json")
    parser.add_argument("--output", type=Path, default=here / "demo_results.png")
    args = parser.parse_args()
    snapshots = load_sample(args.trace)
    results = evaluate(snapshots)
    plot_results(results, snapshots, args.output)
    print("Vehicles     f1 PDR%     f2 PDR%     f3 PDR%     f4 PDR%")
    for index, count in enumerate(COUNTS):
        print(f"{count:8d}" + "".join(f"{results[name][index]:12.2f}"
                                    for name, *_ in OBJECTIVES))
    print(f"Saved {args.output.resolve()}")


if __name__ == "__main__":
    main()
