# Autonomous Segregation System (based on the CacheCons algorithm)

## RoboSim d-model 
### Module: `CacheCons`
 
The top-level module defines:
 
- **Cycle period**: 1 time unit (configurable at code generation).
- **Robotic platform**: `TurtleBot`
- **Controller**: `CacheConsC`
### TurtleBot Interfaces
 
**Provided interfaces** (services offered to the controller):
- `Move` — controls robot movement via `move(cmd: vector(real,2))` (angular and linear velocity)
- `IObjectOps` — provides `DepositObject()` to release objects from the gripper
**Defined interfaces** (events sent to the controller):
- `ObstacleEvents` — `closestDistance` and `closestAngle` from camera
- `IStatus` — `ObjectCarried` boolean event (gripper status)
- `IVisibleClustersCC/CA/TW` — cluster and object data (count, `ClusterData`, `ObjectData`)
- `IPose` — `Pose_O` event with robot position and orientation
---
 
## Controller: `CacheConsC`
 
Contains **state machines**:
 
| State Machine | Role |
|---------------|------|
| `CacheConsS` | Core CacheCons segregation algorithm |
| `ObstacleAvoidance` | Simple obstacle avoidance |
| `RandomWalk` | Exploratory behaviour for object searching |
| `MoveManager` | Priority-based arbitration of movement commands |
| `TargetWatch` | Identifies the closest valid target object |
| `CachePointAssignment` | Manages and updates cache locations per cluster type |
 
**Movement priority** (highest to lowest): Obstacle Avoidance → Cache Construction → Random Exploration
 
---
 
## Core State Machine: `CacheConsS`
 
Implements the CacheCons algorithm through **six composite states**:
 
| State | Behaviour |
|-------|-----------|
| `PU_SCAN` | Scans for clusters; selects an object target; enables random walk and obstacle avoidance |
| `PU_TARGET` | Navigates toward the target using pose and target location data; approaches to pick up |
| `HOMING` | Navigates robot to the appropriate cache point after collecting an object |
| `DE_PUSH` | Slowly pushes the object into the cache location for a timed duration |
| `DE_BACKUP` | Drops the object (`DepositObject()`), then moves the robot backwards |
| `EXILE` | Moves robot away from the cache point before returning to `PU_SCAN` |
 
Key behaviours include:
- **Timer-based transitions** to handle timeouts and cycle advancement via the `exec` event.
- **Accidental pick-up handling** — a direct `PU_SCAN → HOMING` transition if an object is unexpectedly collected.
- **Passive gripper support** — repeated `DepositObject()` calls until the object is confirmed dropped.
---
 
## Key Data Types
- `ClusterData` — cluster size, type, and centroid location
- `ObjectData` — object position, cluster association, and ID
- `vector(real, 2)` — 2D vector for velocity commands

