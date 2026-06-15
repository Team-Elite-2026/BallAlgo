# Other Team Enemy-Robot Localization Notes

This note is about the active code in:

- `other-teams/rcj-soccer-open-gen3/RoboCupOpen`

I am separating:

- exact code behavior
- accuracy assessment / inference

That way the description of the algorithm is factual, and any judgment about "is it accurate?" is clearly marked as an evaluation rather than a claim about measured performance.

## Short answer

Their active code uses lidar as the primary sensor for locating other robots on the field. It does not directly classify "enemy" versus "teammate" inside the lidar module. Instead, the lidar module outputs up to three generic `field_robots`, and higher-level behavior code sometimes treats those as enemies.

My accuracy judgment from the code alone:

- The self-localization part is reasonably well structured for coarse field localization.
- The opponent-detection part is heuristic, not model-based.
- I would trust it for rough obstacle/opponent awareness, not for precise enemy pose estimation.
- I would not call it fully reliable as written, especially for distinguishing teammate vs enemy in the attacker path.

I cannot claim measured real-world accuracy from code inspection alone. I can only say what the code computes and how robust that design looks.

## Files that matter

- Raw lidar packet ingestion: `soccer_robot/interface/undercarriage_bottom_module.py:75-135`
- Lidar scan processing and field-robot extraction: `soccer_robot/interface/lidar/lidar_module.py:71-499`
- Robot pose acceptance from lidar: `soccer_robot/utils/tracker.py:52-63`
- Active attacker use of detected robots: `robot_attacker.py:186-199`, `robot_attacker.py:239-242`
- Active goalkeeper use of detected robots: `robot_miso.py:131-149`, `robot_miso.py:283-295`
- Role logic that treats "any robot seen" as "some other robot is on the field": `robot.py:30-49`
- Important constants: `soccer_robot/constants_robot1.py:76`, `soccer_robot/constants_robot1.py:116`, `soccer_robot/constants_robot1.py:118-120`, `soccer_robot/constants_robot1.py:196`, `soccer_robot/constants_robot1.py:206`

## Exact algorithm

### 1. Raw lidar packets are read from the bottom undercarriage board

`UndercarriageBottomModule._parse_packet()` reads a 152-byte packet and extracts four lidar sub-packets per message. Each sub-packet provides:

- `start_angle`
- `end_angle`
- 12 distance samples

Before forwarding the sub-packet to the lidar module, it adds `LIDAR_OFFSET_ANGLE = 180` degrees to both start and end angles and normalizes them.

This happens in:

- `soccer_robot/interface/undercarriage_bottom_module.py:119-135`
- `soccer_robot/constants_robot1.py:76`

### 2. A full scan is reconstructed from angle wraparound

`LidarModule.on_update()` drains the lidar queue. For each 12-point sub-packet:

- if `start_angle > end_angle`, it adds 360 degrees to `end_angle`
- it linearly interpolates 12 point angles across the packet using `(end_angle - start_angle) / 11`
- it detects scan completion when the next point angle becomes smaller than the previously seen max angle

When that wrap occurs, it considers the accumulated points to be one finished scan and runs `_process_scan_points()`.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:73-120`

### 3. Each lidar point is rotated by the robot heading before processing

For every distance sample, they fetch the robot heading from `UndercarriageModule.get_heading()` and create:

- `scan_point.angle = lidar_angle + heading`
- `scan_point.x = cos(angle) * distance`
- `scan_point.y = sin(angle) * distance`

So the point cloud is not kept in robot-local coordinates. It is rotated into a global field-oriented frame using the compass heading.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:107-120`
- `soccer_robot/interface/lidar/lidar_module.py:15-22`

### 4. They build a Hough space from lidar points to find field walls

Only points with distance in `[300, 3000]` mm contribute to the Hough transform accumulator.

For each such point, they evaluate all line normals from `-90` to `89` degrees in 1-degree steps and vote into a `500 x 180` Hough array using:

- `distance = x * cos(theta) + y * sin(theta)`
- bin index `round(distance / 24) + 249`

So their line representation is quantized to roughly:

- 1 degree in angle
- 24 mm in perpendicular distance

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:58-68`
- `soccer_robot/interface/lidar/lidar_module.py:113-120`

### 5. They estimate robot field position from two dominant orthogonal walls

Inside `_process_scan_points()` they:

1. Find the most-voted Hough line as the "primary line".
2. Search along the same angle column to move that line outward to the furthest bin that still has more than 10 votes.
3. Find the strongest roughly orthogonal line, using a 87 to 93 degree orthogonality test.
4. Again shift that orthogonal line outward to the furthest bin with more than 10 votes.

Then they infer which side of the field the robot is near:

- left vs right from the sign/orientation of the x-like line
- top vs bottom from the sign/orientation of the y-like line

From those two lines they compute:

- `xCoord`
- `yCoord`

using the field size `FIELD_SIZE = Vector2(1820, 2430)`.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:132-248`
- `soccer_robot/constants_robot1.py:206`

### 6. They synthesize the opposite pair of field-border lines

After finding one x-like line and one y-like line, they create two parallel lines corresponding to the opposite walls of the rectangular field.

That gives them four lines total:

- near x wall
- near y wall
- far x wall
- far y wall

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:243-248`

### 7. They label "field structure" points by proximity to any of those four lines

They mark a scan point as belonging to the playfield border if it is within 100 mm of any of the four lines.

So the algorithm assumes that most valid lidar returns should land on the field boundary geometry once self-localization is correct.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:251-260`
- `soccer_robot/interface/lidar/lidar_module.py:438-449`

### 8. They reject near and far points, then verify the whole localization result

Any point outside `(150, 3000)` mm is marked ignored.

Then they compute:

- `field_points_ratio = number_of_points_marked_as_field / number_of_nonignored_points`

If that ratio is below `0.6`, they reject the entire scan:

- `verification = False`
- robot position becomes `(-1, -1)`
- detected field robots are cleared

If the ratio is at least `0.6`, they accept the scan.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:263-282`

Important detail:

- this verification checks only whether the scan looks sufficiently consistent with the field-wall model
- it does not separately verify whether each detected robot cluster is actually a robot

### 9. "Enemy detection" is really leftover-point clustering

Once field-border points are removed, every remaining point with `pointIDs[i] == 0` is treated as a possible non-field object.

They cluster these leftover points in a very simple online way:

- maintain a list of point groups
- compare the new point to each existing group centroid
- if it is within 300 mm, merge it into that group and update the centroid by running average
- otherwise create a new group

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:289-309`

This is not a shape fit, object tracker, or segmentation network. It is just proximity clustering on leftover lidar points.

### 10. They keep up to three largest clusters and call them field robots

After clustering, they:

- sort groups by point count descending
- keep at most the top three groups
- require each accepted group to have more than 5 points

Each accepted cluster centroid is then converted from global point coordinates into field coordinates using the already-estimated field lines and side flags.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:310-319`
- `soccer_robot/interface/lidar/lidar_module.py:453-467`

### 11. They drop detections close to either endline

After converting a cluster centroid to field coordinates, they reject it if:

- `y < 200`, or
- `y > FIELD_SIZE.y - 200`

So a detected cluster near either short edge of the field is discarded.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:314-319`

### 12. The module publishes only up to three positions, with no identity labels

If verification succeeded, the lidar module stores:

- robot self-position `(_positionX, _positionY)`
- number of field robots
- up to three `(x, y)` pairs in `_field_robot_positions`

There is no label saying:

- teammate
- opponent 1
- opponent 2

Only anonymous field-robot positions are published.

This is in:

- `soccer_robot/interface/lidar/lidar_module.py:361-380`
- `soccer_robot/interface/lidar/lidar_module.py:481-494`

## How the active behavior code uses those detections

### Attacker path

The active attacker immediately renames the lidar outputs to `enemy_positions`:

- `self.n_field_robots = LidarModule.get_n_field_robots()`
- `self.enemy_positions = LidarModule.get_field_robot_positions_vec()`

Then it checks whether any detected robot is within 100 mm of a point 100 mm in front of the robot. If yes, it treats that robot as an enemy directly in front and may trigger a duel maneuver after `DUEL_TIME_THRES = 500` ms.

This is in:

- `robot_attacker.py:186-199`
- `robot_attacker.py:239-242`
- `soccer_robot/constants_robot1.py:196`

Important accuracy note:

- the attacker does not filter out its own teammate before calling these positions "enemy"

### Goalkeeper path

The active goalkeeper also reads the lidar field-robot list, but it uses Bluetooth to get the teammate position and rejects any detected field robot that is within 120 mm of the teammate's reported location.

Among the remaining detections, it keeps the one with the largest `y` value as `enemy_position`.

This is in:

- `robot_miso.py:131-149`

Later, if no ball target is preferred, it uses that `enemy_position` to move to a blocking point approximately halfway between:

- the enemy x-position and the goal center x-position
- the enemy y-position and a point near the defending end

This is in:

- `robot_miso.py:283-295`

So the goalkeeper path is more careful than the attacker path, but it is still using a heuristic teammate rejection rule rather than true identification.

### Role logic

The multi-robot role manager resets `no_robots_timer` whenever the lidar module reports any field robot at all.

That means the code interprets:

- "nonzero lidar-detected field robots"

as evidence that another robot is on the field.

This is in:

- `robot.py:30-36`

## What sensors are involved

For enemy-robot position estimation in the active code, the relevant inputs are:

- lidar distance scans
- compass heading, used to rotate scan points into field orientation
- Bluetooth teammate position, but only in the active goalkeeper path, and only as a teammate-rejection heuristic

Notably, the active code does not use:

- front camera robot detection
- mirror camera robot detection
- line sensors

to estimate enemy robot position.

## Accuracy assessment

Everything below is an engineering judgment from the code, not a measured benchmark.

### What looks solid

- The scan-to-field-localization idea is coherent.
- Using dominant orthogonal walls to localize the robot on a rectangular field is reasonable.
- Requiring 60 percent of nonignored points to align with field borders is a meaningful sanity check.
- Converting object centroids into field coordinates using the field model is internally consistent.

### What makes the enemy-position estimate only moderately reliable

1. The lidar module never truly identifies enemies.

It only finds leftover non-wall clusters and exports them as generic `field_robots`. Any later code that calls them enemies is making an additional assumption.

2. The attacker path does not exclude the teammate.

The active attacker treats all lidar-detected field robots as enemies. If the teammate appears in lidar, the attacker can misinterpret that robot as an enemy.

3. The object detection stage is just proximity clustering.

There is no explicit test for:

- robot width
- robot shape
- object persistence across frames
- velocity consistency
- classification confidence

4. The accepted clusters are per-scan raw centroids.

There is no temporal smoothing or multi-frame tracker for enemy positions in the lidar module.

5. Verification is about field-wall consistency, not opponent correctness.

A scan can pass the 60 percent field-border verification and still produce a bad object cluster from leftover clutter or partial returns.

6. Cluster gating is very simple.

The conditions are only:

- centroid-neighbor radius under 300 mm during clustering
- more than 5 points
- not within 200 mm of either endline
- keep at most the three largest groups

That is a useful heuristic, but not a strong discriminator.

7. Self-position acceptance is coarse-gated only by speed.

The tracker accepts a new lidar position if it is valid and its implied speed is under `ROBOT_MAX_SPEED = 2000` mm/s. That helps reject impossible jumps, but it is still a fairly loose gate.

This is in:

- `soccer_robot/utils/tracker.py:52-58`
- `soccer_robot/constants_robot1.py:116`

### My bottom-line judgment

If the question is:

- "Does this code have a real algorithm for estimating where other robots are?"

then yes, definitely.

If the question is:

- "Is it likely accurate enough for coarse tactical behavior like blocking, dueling, or knowing a robot is in front of you?"

then probably yes, under decent lidar visibility.

If the question is:

- "Is it accurate enough to trust as precise opponent localization or reliable teammate-vs-enemy classification?"

then no, not as written.

The strongest accurate summary is:

- good enough to be useful
- not strong enough to be called precise or robust
- especially weak on identity separation in the attacker path
