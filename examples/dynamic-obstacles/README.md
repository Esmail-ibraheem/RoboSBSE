# Dynamic obstacles

Dynamic elements are part of the map format (`robosbse.map/1`), so the dynamic maps live with the other
maps in [`../maps`](../maps):

- [`dynamic-obstacle-course.json`](../maps/dynamic-obstacle-course.json): two walls with 3 m gaps swept by
  guards, cross traffic, a patrol cart on a loop, a pallet present from 15 to 40 s, a restricted
  maintenance area and gravel at half speed.
- [`delivery-center.json`](../maps/delivery-center.json): racks and aisles for multi-robot task allocation.

Fields (all optional; omitted when empty), with one entry of each kind from `dynamic-obstacle-course.json`:

```json
"movingObstacles": [{"label": "gap guard A", "radius": 0.6, "path": [{"x": 12.6, "y": 6.75}, {"x": 14.4, "y": 6.75}],
                     "speed": 0.5, "mode": "bounce", "offset": 0.0}],
"timedObstacles":  [{"label": "pallet", "shape": {"type": "circle", "x": 10.0, "y": 10.0, "r": 0.8},
                     "from": 15.0, "until": 40.0}],
"zones":           [{"kind": "restricted", "label": "maintenance area",
                     "shape": {"type": "rect", "x": 13.0, "y": 8.5, "w": 4.0, "h": 2.5}},
                    {"kind": "terrain", "label": "gravel",
                     "shape": {"type": "rect", "x": 1.0, "y": 15.0, "w": 5.0, "h": 3.5}, "speedFactor": 0.5}]
```

Semantics, and how the planners and the search treat them:
[docs/research/robotics-extensions.md §3](../../docs/research/robotics-extensions.md).
