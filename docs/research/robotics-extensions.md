# Robotics extensions (Phase 5)

Phase 5 adds a planar robotic arm, multi-robot task allocation and dynamic environments. Each is an
ordinary RoboSBSE problem: every algorithm whose representation fits can run on it, results are recorded
the same way, and a conventional method is reported next to the search result. Measured results are in
[phase5-results.md](../experiments/phase5-results.md).

## 1. Planar robotic arm (`org.robosbse.arm`)

### Model

A scene (`robosbse.arm/1`, bundled scenes in `examples/arm/`) defines the workspace, the base, the link
lengths, joint limits, the link radius, the maximum joint speed, the home pose, rectangular and circular
obstacles, a target and optional pick-and-place jobs.

| Aspect | Model |
|---|---|
| Kinematics | Serial planar chain with *relative* joint angles: link `i` points at `q_1 + ... + q_i` (`ArmKinematics.points`) |
| Collisions | Each link is a capsule of radius `linkRadius`. Clearance = smallest distance from points sampled every 2 cm along every link to any obstacle, minus the radius; negative means penetration. Only obstacles count: there is no floor or workspace wall, and the joint limits of the bundled scenes keep the first link above the base |
| Joint limits | A box `[jointMin, jointMax]`; violation = summed excess in radians |
| Motion | Keyframes joined by straight lines in joint space. A segment is a synchronised move lasting `max_i \|dq_i\| / maxJointSpeed`. Collisions are checked so that no joint moves more than 0.02 rad between checks |
| Movement cost | `travel = Σ \|dq_i\|` (rad) |
| Energy | `effort = Σ_segments Σ_i I_i dq_i² / T`, with `I_i = (l_i + ... + l_n)²`: the integral of inertia-weighted squared joint speed for a horizontal arm (no gravity, no friction, no dynamics) |
| Smoothness | `roughness = Σ_k ‖dq_{k+1} − dq_k‖` over interior keyframes (0 for a straight move) |

Analytical inverse kinematics (law of cosines, both elbow solutions) exists for two-link arms. Damped least
squares (`λ = 0.05`, at most 300 iterations) is the numerical method for any number of links.

### Problems

| Problem | Genome | Fitness (minimised) | Target (success) | References |
|---|---|---|---|---|
| `arm-ik` | joint angles in `[−π, π]` | `distance + wC·penetration + wL·limitViolation + wP·Σ\|q − home\|` | distance < tolerance, clearance ≥ 0, no limit violation | analytical IK (2 links), damped least squares from 8 starts |
| `arm-trajectory` | `viaPoints + 1` configurations (the last one is the final pose) | `wR·reach + wC·penetration + wL·limits + wT·travel + wE·effort + wS·roughness` | reach < tolerance, no colliding sample, no limit violation | straight joint-space move to each IK solution; configuration-space A* path (2 links) |
| `arm-pick-place` | random keys: K priorities + K pick-elbow + K place-elbow genes | `time + 100·invalidPoses + wC·penetration + wE·effort`, time including a handling pause at every pick and place | every pose valid and every move planned collision-free | greedy (cheapest next object and elbows); exact optimum by enumeration (K ≤ 5: K!·4^K plans) |

Objectives for multi-objective algorithms: IK (distance, joint motion, collision risk), trajectory (time,
effort, roughness), pick and place (time, effort, collision risk). Collision risk is
`exp(−max(0, clearance) / 0.1 m)`.

*Joint-angle optimisation* is IK with a posture weight (`weightPosture`: prefer configurations close to
home); *energy-efficient movement* is the trajectory problem with `weightEffort > 0` or the NSGA-II front
of time against effort.

### Configuration-space planner (reference)

`ConfigurationSpace` (two-link arms) grids the joint box at 0.02 rad. A cell is free when the arm keeps at
least 1 cm clearance. A* runs on the 8-connected grid with cost `max(|dq_1|, |dq_2|)`, the duration of a
synchronised move, so the grid path is time-optimal. The path is then shortcut greedily wherever the
straight joint-space segment stays collision-free. Node expansions are reported apart from fitness
evaluations. In pick and place, every move between two chosen poses comes from this planner and is
cached per pair. The search therefore decides *task-level* choices (order, elbows), and the planner
supplies collision-free motions.

### Replays

Replays contain frames every 0.05 s: joint positions, clearance and the keyframe segment. Pick-and-place
replays also say which object is carried into each keyframe, so the UI draws objects at their pick point,
in the gripper, or placed. An IK replay shows only the solution configuration, because IK does not plan a
motion.

## 2. Multi-robot task allocation (`multi-robot-allocation`)

A fleet instance (`robosbse.fleet/1`, `examples/fleet/deliveries-12.json`) names a map, the robots'
depots, the planning speed, a service time and transport jobs (pickup, delivery). Each robot carries one
item at a time and, with `returnHome`, ends at its depot.

- **Encoding: random keys**, `2M` genes for M jobs. The integer part of an assignment gene in `[0, R)` is
  the robot; a priority in `[0, 1]` orders each robot's jobs.
- **Cost model**: geodesic distances on the inflated occupancy grid (robot radius + 0.1 m), time =
  distance / speed + 2 × service time per job. Objectives are (makespan, total distance); fitness =
  makespan + `weightDistance` × total distance (default 0.1 s/m).
- **Reference**: greedy list scheduling, which repeatedly appends the job and robot that finish
  earliest.
- **Validation by simulation**: the replay executes every robot's plan with the waypoint follower along A*
  routes. Robots are simulated **independently**, so they do not see or avoid each other. Afterwards the
  trajectories are compared at equal times (0.1 s steps). Every interval in which two robots come closer
  than two radii plus 0.1 m is reported as a **conflict**, with time, place and closest distance.
  Conflicts are measured, not resolved: coordination (priorities, reservations, replanning) is not
  implemented.

## 3. Dynamic environments

Maps (`robosbse.map/1`) may contain, in addition to static obstacles:

| Element | Semantics |
|---|---|
| `movingObstacles` | Circle moving along a polyline at constant speed, `bounce` (back and forth) or `loop` (closed polygon), with a start offset. The position is a pure function of time, so episodes stay deterministic. Moving obstacles do not react to the robot |
| `timedObstacles` | Circle or rectangle present during `[from, until)`; `until` may be omitted (it stays) |
| `zones` | `restricted`: the robot may enter, but the time its body overlaps the zone is measured (`restrictedTime`) and route optimisation adds 20 per second to the fitness and counts it as a constraint violation. `terrain`: the achievable speed is multiplied by `speedFactor` while the robot's centre is inside |

The simulator evaluates sensing (ray casting), clearance and collisions at the current simulation time.
Hitting a moving or timed obstacle is a collision that ends the episode (`dynamicCollision` metric).
Static maps produce bit-identical results to the earlier phases (checked on four recorded experiments and
by `DynamicWorldTest`).

The grid planners (A*, Dijkstra, distance fields) plan on the **static** map: restricted zones are
blocked, and terrain multiplies step costs by `1/speedFactor`, but moving and timed obstacles are ignored.
On `dynamic-obstacle-course`, the A* route therefore collides with a gap guard. Search, by contrast, sees
the moving obstacles through the simulation-based fitness, including how its waypoints time the
crossings.

Built-in maps: `dynamic-obstacle-course` (two walls with 3 m gaps swept by guards, cross traffic, a
patrol cart, a pallet present from 15 to 40 s, a restricted maintenance area, gravel at half speed) and
`delivery-center` (racks and aisles for multi-robot allocation).

**Testing oracle.** The failure-discovery problem still uses static arenas. With moving obstacles, a
collision can be unavoidable for any controller, so the argument that every safety violation is a
controller defect ([formulations, §3](problem-formulations.md)) would have to be refined first. Dynamic
maps are therefore not offered for failure discovery.

## 4. Limitations

- The arm is kinematic: no gravity, torque limits or dynamics. Effort is a proxy, not measured energy.
- Links collide only with obstacles, not with each other, a floor or the workspace border.
- The configuration-space planner exists only for two-link arms. For three links, the trajectory references
  are straight moves to damped-least-squares solutions.
- Multi-robot replays expose conflicts but do not resolve them. The cost model ignores waiting and
  congestion.
- Grid planners ignore moving obstacles. A time-aware planner (space-time A*, velocity obstacles) is not
  implemented.
- The Environment Editor preserves dynamic elements of a map but cannot create or edit them. They are
  written as JSON.
