# LOGIC forwarding on a Yanbu trace sample

A small, standalone Python demonstration of four forwarding objectives on
SUMO-generated vehicle mobility from Yanbu Industrial City, Saudi Arabia.
Running the script produces one PNG with four subplots. Each subplot contains
one PDR curve for one objective. No CSV is written.

## Run

Use Python 3.10 or later:

```sh
python -m pip install -r requirements.txt
python demo.py
```

The result is `demo_results.png` next to the script. The numerical PDR values
are also printed in the terminal. No SUMO installation, Origin project,
simulator framework or other project folder is needed.

Optional paths:

```sh
python demo.py --trace trace_sample.json --output demo_results.png
```

Default input and output paths are relative to the script, so the script can
also be launched from another working directory.

## What the example demonstrates

The four panels implement the stated objective functions:

| Objective | Local relay score |
|---|---|
| f1, greedy | Negative current distance from the candidate to the destination. |
| f2, predicted greedy | Mean normalized geographic progress over predicted positions. |
| f3, hard-in-range greedy | f2, admitting only candidates whose link to the sender stays within range at every prediction sample. |
| f4, soft-in-range greedy | Equal-weight sum of normalized predicted link retention and geographic progress. |

Candidates must be current neighbors, provide strict geographic progress and
not have been visited. An in-range destination is served directly. An empty
candidate set or rejection of every candidate terminates the route normally.
Ties use stable vehicle-ID order. Min-max normalization returns zero when all
candidate values are equal.

For each candidate, prediction lasts the smaller of 3 seconds and its estimated
packet transmission time. Samples include time zero and every 0.25 seconds up
to that horizon. Positions are extrapolated using the snapshot speed and
heading. The f4 retention term measures the signed change in sender-to-relay
separation. It is not a binary connectivity probability.

## Trace and comparison

`trace_sample.json` contains 612 vehicle records across five snapshots from
one SUMO traffic run, seed 101, generated with SUMO 1.28.0. The road network was
imported from OpenStreetMap. These are simulated vehicle trajectories on a
real road network, not measured vehicle movements.

- Region: 600 x 600 m, SUMO x in [4900, 5500), y in [9400, 10000) metres.
- Snapshot times: 420, 660, 2100, 2400 and 3660 seconds.
- Selection: five evenly spaced indices among the 19 eligible parent times,
  without selecting on routing outcomes.
- Coordinates: original SUMO positions, without grid quantization or relocation.
- Heading: SUMO degrees clockwise from north, converted to Cartesian velocity
  by the script.
- Endpoints: two previously frozen source/destination pairs per snapshot.

At each count from 30 to 100 vehicles, the same endpoint vehicles are retained
and the other vehicles form nested populations using random seed 20261020.
All four objectives receive identical inputs. Ten endpoint cases are evaluated
at every count. The sample records the source mapping filenames and SHA-256
hashes, together with the source plan hash, for provenance. The source trace
is the Yanbu seed-101 `trace.xml` from the existing experiment.

## Packet model and PDR

| Setting | Value |
|---|---|
| Communication range | 50 m |
| Packet payload | 1024 bytes, converted to 8192 bits |
| Bandwidth | 1 MHz |
| Transmit power | 1 mW |
| Noise spectral density | 1e-10 W/Hz |
| Reference channel gain and loss exponent | 1 at 1 m, exponent 2 |
| Packet generation | 0, 3, 6, 9, 12, 15 and 18 seconds |
| Observation duration | 20 seconds |
| Packet TTL | 15 seconds |
| Maximum forwarding links | 20 |

A fixed path is constructed from the initial snapshot using local relay
choices. Each packet is then evaluated independently along that path. Nodes
move continuously at their initial speed and heading. Each hop checks range
at transmission start and uses the distance-based Shannon rate for its
transmission time. There is no queueing, interference, retransmission or route
repair, and future SUMO trajectories are not replayed. Only packets reaching
the intended destination by both the TTL and observation deadlines count as
delivered. Incomplete routes contribute zero deliveries.

PDR is 100 times delivered packets divided by all offered packets. There are
70 offered packets per count and objective. This is also the arithmetic mean
of the ten case PDRs because every case offers seven packets. Curves use the
computed points directly, without smoothing or monotonic adjustment.

This is a newly factored implementation of the four objective definitions with
explicit SI units and a deliberately small packet model. It does not import
the original experiment code or reproduce the paper's complete evaluation.
All panels use the same axis limits. With this small sample, two objectives
can select identical routes and produce identical curves.

## Files and attribution

- `demo.py`: objectives, routing, packet evaluation and plotting.
- `trace_sample.json`: the selected trace records and endpoint pairs.
- `requirements.txt`: the two Python dependencies.
- `demo_results.png`: generated example output.

Road network source: [OpenStreetMap](https://www.openstreetmap.org/),
map data copyright OpenStreetMap contributors, available under the
[Open Database License](https://www.openstreetmap.org/copyright).
Traffic simulation: [SUMO](https://eclipse.dev/sumo/).

The example was verified using NumPy 2.3.5 and Matplotlib 3.10.6.
