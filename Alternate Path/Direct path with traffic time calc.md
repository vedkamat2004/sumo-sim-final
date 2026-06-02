# How We Estimate Direct Path Time With Traffic

## Goal

When the hero takes the alternative route, we estimate:
"How long would the hero have taken on the direct route?"

This is a counterfactual estimate (something that did not actually happen).

---

## Why This Is Hard

- The hero did not actually drive the direct route.
- Traffic changes every second.
- A small timing change can cause very different queue delays.

---

## We Use 3 Methods (Best to Worst)

1. Method 1: Routing decision ETA (best)
2. Method 2: Sampled traffic estimate (good fallback)
3. Method 3: Theoretical free-flow time (last fallback)

---

## Method 1 (Best): Routing Decision ETA

### Simple idea

At the exact moment congestion is detected, the routing policy asks:

- "If I stay on direct path from here, how many seconds left?"
- "If I switch to alternate path, how many seconds left?"

That direct-path number is saved and later reused as the main counterfactual time.

### Where in code

- `controller/DynamicReroutePolicy.py`
- Direct ETA computed at: `direct_eta = self._estimate_path_time(vehicle.current_edge, self.direct_path)`
- Saved when reroute triggers: `self.reroute_decision_direct_eta = direct_eta`

### How direct ETA is computed (inside `_estimate_path_time`)

For each remaining edge on the direct route:

1. Read edge length from map
2. Read live mean speed from SUMO (`traci.edge.getLastStepMeanSpeed`)
3. Read live occupancy from SUMO (`traci.edge.getLastStepOccupancy`)
4. Build congestion penalty
5. Compute effective speed
6. Add edge time = `edge_length / effective_speed`

Then sum all edge times:
`direct_eta = edge1_time + edge2_time + ...`

### Formula used

- `occ_ratio = occ_raw / 100` (if occupancy is in percent)
- `penalty = 1 + max(0, occ_ratio - 0.10) * 5`
- `effective_speed = max(0.3, mean_speed / penalty)`
- `edge_time = edge_length / effective_speed`

### Why Method 1 is strongest

- Uses live traffic at decision time
- Uses the exact same logic used for rerouting
- Captures remaining path from current hero position

### Limitation

- Still a forecast from that instant, not guaranteed ground truth
- If traffic changes sharply after decision, real time may differ

Typical error: around plus/minus 10 to 20 percent.

---

## Method 2: Sampled Traffic Estimate

- While hero is active, sample direct-path travel times each step using `getAdaptedTraveltime`.
- Average samples per edge, then sum across edges.
- Report expected, best-case, and worst-case values.

Typical error: around plus/minus 15 to 30 percent.

---

## Method 3: Theoretical Free-Flow Time

- Compute `distance / free_flow_speed`.
- Ignores congestion and queueing.

Typical error: around plus/minus 50 to 80 percent.

---

## How To Read Output

- If output says Method 1: highest confidence
- If output says Method 2: use uncertainty range too
- If output says Method 3: treat as rough lower bound only

---

## Final Takeaway

Method 1 is the best practical estimate in this system because it is calculated at the reroute decision moment with live traffic data and the same logic that chooses the route.

For exact ground truth, run two simulations:

1. Hero forced on alternative route
2. Hero forced on direct route
   and compare actual times.
