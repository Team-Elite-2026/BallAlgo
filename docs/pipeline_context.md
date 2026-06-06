### Step 1: Global Sensor Fusion & State Estimation (Pi Loop)

*Variable frequency, triggered asynchronously by LiDAR or Camera updates.*

The Pi maintains the ground-truth state vector of the field using a Global Extended Kalman Filter (EKF). Because we treat the Teensy's IMU as the absolute authority on heading, the EKF tightly couples the high-frequency local odometry with the low-frequency global LiDAR updates and camera bearing arrays.

- State Vector: $\mathbf{x} = [x_r, y_r, \theta_r, v_{rx}, v_{ry}, \omega_r]^T$

Have Another smaller Linear kalman filter for ball state

- $\mathbf{x}_k = [x_b,y_b, v_{bx}, v_{by}]$
- camera only gives x,y of ball
- Build transition matrix F with $\gamma$=0.95 and approximate the friction as viscous friction
- use a friction coefficient to define ball’s deceleration:
- have high measurement noise for velocity

### a. The High-Low Frequency Fusion (Mouse + LiDAR + IMU)

The EKF handles the drastic difference in sensor speeds ($1\text{kHz}$ vs $15\text{Hz}$) through a Predict-Update cycle:

- The Prediction Step ($1\text{kHz}$ Mouse + IMU): Between LiDAR frames, the robot relies entirely on dead-reckoning. The Teensy streams its raw local mouse velocities ($v_{x, \text{local}}, v_{y, \text{local}}$) and its highly trusted IMU heading ($\theta_{\text{imu}}$). The EKF integrates this high-speed data to instantly predict the robot's new global position:
    
    $\begin{bmatrix} \dot{x}_r \\ \dot{y}_r \end{bmatrix} = \begin{bmatrix} \cos\theta_{\text{imu}} & \sin\theta_{\text{imu}} \\ -\sin\theta_{\text{imu}} & \cos\theta_{\text{imu}} \end{bmatrix} \begin{bmatrix} v_{x, \text{local}} \\ v_{y, \text{local}} \end{bmatrix}$
    
    - This rotation matrix accounts for our non-standard robot angle convention btw
- The Map Update ($15\text{Hz}$ LiDAR): When a LiDAR packet arrives ($[x_{\text{lidar}}, y_{\text{lidar}}]$), the EKF compares the absolute field coordinates to the mouse's prediction, calculating the residual and snapping the global position ($x_r, y_r$) back to reality to erase accumulated drift.

### b. Target Tracking & Dual-Goal Triangulation (Vision)

- The Ball (NN + `atan2`): The camera pipeline detects the ball. `atan2` computes a highly accurate pixel-based angle ($\phi_{\text{ball}}$). The pre-trained Neural Network processes the pixel data to output a true real-world distance ($d_{\text{nn}}$). The Pi converts this to a local vector:
    
    $x_{b, \text{local}} = d_{\text{nn}} \sin(\phi_{\text{ball}})$
    
    $y_{b, \text{local}} = d_{\text{nn}} \cos(\phi_{\text{ball}})$
    
    This local vector is rotated by $\theta_{\text{imu}}$ and added to the robot's global position to yield absolute ball coordinates ($x_b, y_b$).
    
- Dual-Goal Bearing-Only EKF Update (Dynamic Occlusion): Because camera depth perception is noisy at long ranges, the distance to the goal is derived strictly from the map. However, the camera's highly accurate *angles* to the goals are fed back into the EKF to correct $(x,y)$ lateral drift.
    
    If the vision network outputs a visibility certainty score (based on how much of the goal is visible)) $c \in [0, 1]$, the measurement variance $\mathbf{R}_i$ dynamically scales to handle occlusion:
    
    $\mathbf{R}_i = \sigma_{\text{base}}^2 + K_{\text{penalty}} \cdot (1 - c)^2$
    
    Now you can compute the predicted goal angle from the EKF:
    
    - $h_i(\mathbf{x}) = \text{atan2}( x_{gi} - x_r, y_{gi} - y_r) - \theta_r$
        - $y_\text{gi},g_\text{gi} = \text{fixed absolute goal pos where }i \in \{\text{blue}, \text{yellow}\}$
    - Pi feeds the residual ($z_{\text{cam}} - h(\mathbf{x})$)  to the EKF
        - make sure to account for 360-0 edge case
        - create jacobian
    - make sure to input $\bf{R}_i$ into the EKF
    - Aside: if you detect very low certainty (c<threshold) from camera, dont even bother doing the rest of the computation
    
    If a goal is heavily occluded (low $c$), the variance blows up, and the Kalman Gain safely ignores the bad visual data.
    

### Step 2: Trajectory Look-Ahead Pipelining (Pi Loop)

To eliminate the $5\text{–}15\text{ ms}$ processing and serial transmission latency gap:

1. When a replan triggers at $t_{\text{replan}}$, the Pi calculates: $t_{\text{lookup}} = t_{\text{replan}} + \text{pipeline\_latency\_us}$.
2. the latency is a constant we have to find by timing compute time on the pi
1. The Pi queries the currently executing Action Chunk at $t_{\text{lookup}}$ to extract the predicted global state.
2. compute error matrix:
    
    $$
    \mathbf{e}_{\text{tracking}} = \begin{bmatrix} x_{\text{EKF}} \\ y_{\text{EKF}} \end{bmatrix} - \begin{bmatrix} x_{\text{chunk\_predicted\_past}} \\ y_{\text{chunk\_predicted\_past}} \end{bmatrix}, \quad e = \|\mathbf{e}_{\text{tracking}}\|
    $$
    
3. find EKF weighting term:
    
    $$
    \alpha = \exp\left( -\left(\frac{e}{\sigma_{\text{track}}}\right)^2 \right)
    $$
    
    1. sigma is acceptable error standard deviation (3-5cm probably)
4. Compute Interpolated starting pose:
    
    $$
    \mathbf{x}_{\text{start}} = \alpha \cdot \mathbf{x}_{\text{chunk\_predicted}} + (1 - \alpha) \cdot \left( \mathbf{x}_{\text{EKF}} + \mathbf{v}_{\text{EKF}} \cdot \text{pipeline\_latency} \right)\\\mathbf{v}_{\text{start}} = \alpha * v_{\text{chunk\_pred}} + (1 - \alpha) * v_{\text{ekf}}\\\mathbf{a}_{\text{start}} = \alpha * a_{\text{chunk\_pred}}*(1-\alpha)*[0,0,0]
    $$
    
    1. velocities and accelerations include its rotational counterparts
5. This state becomes the starting node for the $A^*$ search.

#### Step 2.5. Calculate Strike Pose

1. Use Iterative Convergence to find time of interception
    - $T_{\text{int}} = \frac{\sqrt{(x_b - x_{\text{start}})^2 + (y_b - y_{\text{start}})^2}}{V_{\max}}$
    - $N = T_{\text{int}}* fps_{\text{cam}}$
    
    $$
    x_{\text{int}} = x_b + v_{bx} \cdot \Delta t_{\text{cam}} \left( \frac{1 - \gamma^N}{1 - \gamma} \right)\\y_{\text{int}} = y_b + v_{by} \cdot \Delta t_{\text{cam}} \left( \frac{1 - \gamma^N}{1 - \gamma} \right)
    $$
    
    - Run a lower resolution A* search with the cost from above (from $X_\text{start}$ to target pose)
    - $T_\text{path}$ = g(target node) of A* cost
    - $T_\text{int} = T_\text{path}$
    - repeat until convergance unless $T_\text{int}$ > $K_\text{max\_time}$, which in this case just clamp at K_max_time
- Calculate the goal angle using the robot's global $(x, y)$ position and the static field coordinates of the goal ($\mathbf{p}_{\text{goal}}$).
- Strike Pose: $\bf{x}_t = [x_t, y_t, \theta_t, v_x, v_y, \omega_t]^T$
- Behind the ball:
    - $\mathbf{p}_t = \begin{bmatrix} x_t \\ y_t \end{bmatrix} = \mathbf{p}_b - d_{\text{offset}}\hat{\mathbf{u}}_g$
        - the ball position is the future predicted intercept ball position
    - $\theta_t = \theta_g$
    - add desired target velocities
- This will be the final point after the A* discrete trajectory

### Step 3: Global Kinematic $A^*$ (Pi Loop)

The $A^*$ planner operates in a 3D configuration space: 

$\text{Node} = (x, y, \theta_{\text{index}})$.

### 1. 3D State Space Setup & Obstacle Inflation

- Grid discretization: The physical $182\text{ cm} \times 132\text{ cm}$ field is broken into a spatial grid (typically $5\text{ cm}$ cells). The heading dimension is split into 8 discrete directions: $\theta_{\text{index}} \in \{0, 1, \dots, 7\}$, mapping to increments of $45^\circ$ ($0^\circ, 45^\circ, 90^\circ, \dots$).
- inflate the ball as an obstacle
- (optional) Dynamic obstacle inflation: Enemy robot coordinates $(x_e, y_e)$ are pulled from the global EKF state vector. To prevent physical collisions, every cell within an inflation radius $R_{\text{inflate}}$ is marked as strictly impassable:

$$
R_{\text{inflate}} = R_{\text{you}} + R_{\text{enemy}} + \text{safety\_margin}
$$

### 2. Asymmetric Move Angle Evaluation & Step Cost ($g$)

When expanding from a parent node $A(x_A, y_A, \theta_A)$ to a neighbor node $B(x_B, y_B, \theta_B)$, the algorithm evaluates the physical travel angle relative to the robot's heading configuration:

1. Calculate the global travel vector heading: $\phi_{\text{global}} = \text{atan2}(x_B - x_A, y_B - y_A)$.
2. Map this vector into the robot's expected local body frame: $\phi_{\text{local}} = \phi_{\text{global}} + \theta_{\text{A}}$(make sure to handle edge case)
3. Compute the directional velocity ceiling $V_{\max}(\phi_{\text{local}})$ 
    
    ```cpp
    #include <cmath>
    #include <algorithm>
    
    float calculate_dynamic_max_linear_velocity(float target_phi) {
        float max_velocity = 999.0f; 
    
        // 1. Convert the datasheet's no-load RPM to rad/s
    		float max_wheel_omega_noload = (datasheet_no_load_rpm * 2.0f * M_PI) / 60.0f; 
    		
    		// 2. Subtract your empirical load compensation (also in rad/s)
    		float max_wheel_omega_loaded = max_wheel_omega_noload - load_comp_omega;
    		
    		// 3. Convert to maximum linear wheel velocity for your kinematic limits
    		float max_wheel_vel_mps = max_wheel_omega_loaded * r_wheel;
    
    		for (int i = 0; i < 4; i++) {
    		        // Project the chassis translational velocity onto this specific wheel's axis
    		        // Note: sin(a)sin(b) + cos(a)cos(b) is mathematically equivalent to cos(a - b)
    		        float proj_trans = std::sin(alpha_wheel[i]) * std::sin(target_phi) + 
    		                           std::cos(alpha_wheel[i]) * std::cos(target_phi);
    		        
    		        // Prevent division by zero if the wheel doesn't contribute to this heading
    		        if (std::abs(proj_trans) > 0.001f) {
    		            // The max chassis velocity is limited by the wheel that has to spin the fastest.
    		            // We scale the max wheel speed by the inverse of its projection.
    		            float v_limit = max_wheel_vel_mps / std::abs(proj_trans);
    		            
    		            // Constrain the overall max velocity to the most restrictive wheel limit
    		            max_velocity = std::min(max_velocity, v_limit);
    		        }
    		    }
    		
    		    return max_velocity;
    }
    ```
    
4. Calculate the Incoming Travel Angle: The global direction of travel from parent node (P) to A: $\phi_{\text{in}} = \text{atan2}(x_A - x_P, y_A - y_P)$
5. calculate the Outgoing Travel Angle : The global direction of travel from A to B: $\phi_{\text{out}} = \text{atan2}(x_B - x_A, y_B - y_A)$
6. $\Delta\phi = \text{angle\_diff} (\phi{\text{out}}, \phi_{\text{in}})$ (please pleaee account for edge cases bruh very hard to catch bug)
7. $\Delta\theta = \theta_B - \theta_A$
8. Accumulate the total cost using time as the cost metric, adding a penalty multiplier ($K_\text{curve}$) for changes in curves:

$$
g(B) =
g(A) +
\frac{\text{Distance}_{A \to B}}{V_{\max}(\phi_{\text{local}})} +
K_{\text{rot}} \cdot \lVert \Delta\theta \rVert + K_{\text{curve}} \cdot |\Delta\phi|
$$

### 3. remaining Time-To-Go Heuristic ($h$)

To remain strictly underestimating the true cost, the heuristic assumes the robot can travel in a perfectly straight line toward the goal at its absolute highest theoretical physical speed limit ($V_{\max} = \max(V_{\max, x}, V_{\max, y})$):

> Remember that $V_{\text{max}}$ should be your MAX velocity, Independent of drive angle. so thet is max velocity driving straight in our case
> 

$$
h(n) = \frac{\text{Euclidean Distance to Target}}{V_{\max}}
$$

So your total cost function for a given node n is:

$$
f(n) = g(n) + h(n)
$$

- the h(n) is needed to properly run Dijkstra's Algo

### Step 4: Parametric Spline Slicing & Asymmetric S-Curve Velocity Profiling (Pi Loop)

Once $A^*$ returns the 3D discrete waypoints, they are smoothed into an executable trajectory. Rather than utilizing trapezoidal steps an S-curve profile builds smooth trapezoids so there is always a constant jerk during acceleration

### 1. Spline Fitting and Geometry Extraction

The discrete waypoints are used as control anchors to construct a continuous parametric cubic spline:

$\mathbf{S}(s) = [x(s), y(s), \theta(s)]^T$, where $s \in [0, 1]$ represents the progress along the curve.

Velocity Constraints:

1. $S'(0) = [v_{\text{start, } x}, \ v_{\text{start, } y}, \ \omega_{\text{start}}]^T$
2. $S'(1) = [v_\text{end},v_ \text{end},\omega_ \text{end}]^T$

### 3. Time-Optimal S-Curve Generation (The 7-Phase Profile)

The Pi runs a forward/backward integration pass over the spline segments. Instead of jumping instantly to $A_{\max}$ (creating a step-change in motor current), the profile ramps acceleration linearly up and down using fixed maximum jerk constraints ($J_{\max}$). This splits every velocity transition into distinct phases:

$V_0 = \|\mathbf{v}_{\text{start}}\| \\\omega_0= \omega_\text{start}\\A_0 = \|\mathbf{a}_{\text{start}}\|\\\alpha_0 = \alpha_{\text{start}}$

**Step 1: The Geometric Pre-Computation Pass**
Before any time-integration occurs, the Pi must evaluate the geometric spline to build a map of the absolute physical limits at every point. For any point  $s \in [0,1]$, it calculates the spatial derivatives: $x'(s) = \frac{dx}{ds} \quad y'(s) = \frac{dy}{ds} \quad \theta'(s) = \frac{d\theta}{ds} \quad \kappa(s) = \frac{|x'(s)y''(s) - y'(s)x''(s)|}{(x'(s)^2 + y'(s)^2)^{3/2}}$
Because the spatial derivatives strictly define the ratio of translation to rotation, the Pi determines the instantaneous path velocity ceiling $\dot{s}_{\max}(s)$ at every point. This ceiling is the hard minimum of the motor back-EMF limit (electrical headroom) and the lateral traction limit (centripetal friction).

**Step 2: The Forward Pass (Phases 1–4)**
Starting at s=0 with initial path velocity $\dot{s}_0$, the integrator steps forward by ds. At each step, it calculates the available dynamic acceleration $\ddot{s}_{\max}(s)$ based on the robot's state/spline state at the previous step.
• **Phase 1: Ramping Acceleration:** Path jerk is set to $J_{\max}$. Path acceleration $\ddot{s}$ builds linearly toward $\ddot{s}_{\max}(s)$.
• **Phase 2: Constant Acceleration:** Path jerk drops to 0. Path acceleration is sustained at $\ddot{s}_{\max}(s)$, continually adjusting to "ride the absolute physical limit" of the motors and traction.
• **Phase 3: Ramping Down Acceleration:** As the integrated forward velocity approaches the pre-calculated ceiling $\dot{s}_{\max}(s)$, path jerk hits $-J_{\max}$ to smoothly round off the curve.
• **Phase 4: Constant Cruise:** Path acceleration is held at 0. The forward velocity rides exactly on the $\dot{s}_{\max}(s)$ ceiling.

**Step 3: The Backward Pass (Phases 5–7)**
The Pi jumps to $s=1$, sets the target exit velocity $\dot{s}_{\text{end}}$, and integrates backward by $-ds$.
• **Phases 5–7: Deceleration S-Curve:** The integrator works backward, strictly querying the dynamic deceleration boundary to find the maximum allowable braking force $\ddot{s}_{\min}(s)$ by using the values from the future spline step. Crucially, this backward pass is *also* bounded by the pre-calculated $\dot{s}_{\max}(s)$ ceiling. By strictly adhering to the ceiling, the backward pass automatically scrubs momentum before every sharp corner, creating a braking S-Curve prior to every local minimum in the path limit.

**Step 4: The Intersection**

The true, final, mathematically safe velocity profile is strictly defined as the intersection of the two passes:

$\dot{s}_{\text{final}}(s) = \min(\dot{s}_{\text{fwd}}(s), \dot{s}_{\text{bwd}}(s))$
The actual real-world commands are then derived directly from the calculated velocities:

```cpp
s_ddot =
    (s_dot_final_next * s_dot_final_next - s_dot_final_current * s_dot_final_current)
  / (2.0f * ds);

vx = dx_ds * s_dot_final;
vy = dy_ds * s_dot_final;
omega = dtheta_ds * s_dot_final;

ax    = x''(s)     * s_dot^2 + x'(s)     * s_ddot;
ay    = y''(s)     * s_dot^2 + y'(s)     * s_ddot;
alpha = theta''(s) * s_dot^2 + theta'(s) * s_ddot;
```

run this in a `for` loop over s from 0 to 1 **before** the profiler runs to build your ceiling array.

```cpp
#include <cmath>
#include <algorithm>

// Experimental lateral traction limit (m/s^2) - tune this based on your carpet/wheels
const float a_max_lateral = 1.5f; 

float calculate_path_velocity_ceiling(float dx_ds, float dy_ds, float dtheta_ds, float kappa) {
    float s_dot_limit = 9999.0f;

    // 1. Calculate Motor Back-EMF Limit
        // 1. Convert the datasheet's no-load RPM to rad/s
		float max_wheel_omega_noload = (datasheet_no_load_rpm * 2.0f * M_PI) / 60.0f; 
		
		// 2. Subtract your empirical load compensation (also in rad/s)
		float max_wheel_omega_loaded = max_wheel_omega_noload - load_comp_omega;
		
		// 3. Convert to maximum linear wheel velocity for your kinematic limits
		float max_wheel_vel = max_wheel_omega_loaded * r_wheel;
    
    for (int i = 0; i < 4; i++) {
        // Find the geometric ratio K_i for this wheel
        // Matches the (0,1) -> 0 deg convention
        float proj_trans = std::sin(alpha_wheel[i]) * dx_ds + std::cos(alpha_wheel[i]) * dy_ds;
        float proj_rot = -R_chassis * dtheta_ds;
        
        float K_i = proj_trans + proj_rot;
        
        // If K_i is non-zero, this wheel's speed limits s_dot
        if (std::abs(K_i) > 0.001f) {
            float s_dot_wheel_limit = max_wheel_vel / std::abs(K_i);
            s_dot_limit = std::min(s_dot_limit, s_dot_wheel_limit);
        }
    }

    // 2. Calculate Traction (Centripetal) Limit
    if (std::abs(kappa) > 0.001f) {
        float V_curve_max = std::sqrt(a_max_lateral / std::abs(kappa));
        float V_path_ratio = std::sqrt(dx_ds * dx_ds + dy_ds * dy_ds);
        
        if (V_path_ratio > 0.001f) {
            float s_dot_traction = V_curve_max / V_path_ratio;
            s_dot_limit = std::min(s_dot_limit, s_dot_traction);
        }
    }

    return s_dot_limit;
}
```

Run this dynamically **during** the Forward and Backward passes to find your acceleration bounds.

```cpp
#include <cmath>
#include <algorithm>

struct RobotState {
    float vx, vy, omega;
};

struct DynamicLimits {
    // Positive path acceleration limit
    float max_s_ddot_accel;

    // Positive magnitude of path deceleration limit
    float max_s_ddot_decel;
};

// Physical constants
const float alpha_wheel[4] = {
    -2.5307f, -0.6109f, 0.6109f, 2.5307f
};

const float R_chassis = 0.09f;
const float r_wheel = 0.025f;
const float V_bus = 12.0f;

// Motor constants
const float kS = 0.5f;   // Static friction voltage
const float kV = 0.08f;  // Back-EMF constant, V per rad/s
const float kA = 0.02f;  // Acceleration constant, V per rad/s^2

const float EPS = 0.001f;

// Total available acceleration from traction/grip.
// This can replace both a_max_lateral and a_max_traction
// in the simplified traction model.
const float a_max_grip_accel = 1.5f;  // m/s^2

DynamicLimits calculate_dynamic_limits(
    float dx_ds,
    float dy_ds,
    float dtheta_ds,

    float d2x_ds2,
    float d2y_ds2,
    float d2theta_ds2,

    float s_dot,

    RobotState current_state,
    float kappa
) {
    DynamicLimits limits;

    limits.max_s_ddot_accel = 999.0f;
    limits.max_s_ddot_decel = 999.0f;

    float s_dot_sq = s_dot * s_dot;

    // ------------------------------------------------------------
    // 1. Motor voltage limits, converted directly to s_ddot limits
    // ------------------------------------------------------------
    for (int i = 0; i < 4; i++) {
        // --------------------------------------------------------
        // 1A. Current wheel velocity
        // --------------------------------------------------------
        float current_v_wheel =
            std::sin(alpha_wheel[i]) * current_state.vx
          + std::cos(alpha_wheel[i]) * current_state.vy
          - R_chassis * current_state.omega;

        float current_w_motor = current_v_wheel / r_wheel;

        // --------------------------------------------------------
        // 1B. Available motor voltage headroom
        // --------------------------------------------------------
        float v_back_emf = kV * current_w_motor;

        float friction_v = 0.0f;
        if (current_w_motor > 0.01f) {
            friction_v = kS;
        } else if (current_w_motor < -0.01f) {
            friction_v = -kS;
        }

        float max_volt_pos = V_bus - friction_v - v_back_emf;
        float max_volt_neg = -V_bus - friction_v - v_back_emf;

        // Convert voltage headroom to wheel linear acceleration limits
        float a_wheel_max_pos = (max_volt_pos * r_wheel) / kA;
        float a_wheel_max_neg = (max_volt_neg * r_wheel) / kA;

        // --------------------------------------------------------
        // 1C. Wheel acceleration model
        //
        // wheel_vel_i = K_i * s_dot
        //
        // wheel_accel_i =
        //      K_i * s_ddot
        //    + C_i * s_dot^2
        //
        // K_i comes from first derivatives.
        // C_i comes from second derivatives.
        // --------------------------------------------------------

        float K_i =
            std::sin(alpha_wheel[i]) * dx_ds
          + std::cos(alpha_wheel[i]) * dy_ds
          - R_chassis * dtheta_ds;

        float C_i =
            std::sin(alpha_wheel[i]) * d2x_ds2
          + std::cos(alpha_wheel[i]) * d2y_ds2
          - R_chassis * d2theta_ds2;

        // Acceleration already required just because the path is curving
        // or theta is changing nonlinearly with s.
        float a_wheel_curve_term = C_i * s_dot_sq;

        if (std::abs(K_i) > EPS) {
            // Solve:
            //
            // a_wheel_max_neg <= K_i * s_ddot + a_wheel_curve_term
            //                   <= a_wheel_max_pos
            //
            // Therefore:
            //
            // s_ddot <= (a_wheel_max_pos - a_wheel_curve_term) / K_i
            // s_ddot >= (a_wheel_max_neg - a_wheel_curve_term) / K_i

            float s_ddot_bound1 =
                (a_wheel_max_pos - a_wheel_curve_term) / K_i;

            float s_ddot_bound2 =
                (a_wheel_max_neg - a_wheel_curve_term) / K_i;

            float local_s_ddot_max =
                std::max(s_ddot_bound1, s_ddot_bound2);

            float local_s_ddot_min =
                std::min(s_ddot_bound1, s_ddot_bound2);

            // Positive s_ddot = accelerate forward along the path
            if (local_s_ddot_max > 0.0f) {
                limits.max_s_ddot_accel =
                    std::min(limits.max_s_ddot_accel,
                             local_s_ddot_max);
            } else {
                limits.max_s_ddot_accel = 0.0f;
            }

            // Negative s_ddot = decelerate/brake along the path
            if (local_s_ddot_min < 0.0f) {
                limits.max_s_ddot_decel =
                    std::min(limits.max_s_ddot_decel,
                             std::abs(local_s_ddot_min));
            } else {
                limits.max_s_ddot_decel = 0.0f;
            }
        } else {
            // If K_i is near zero, this wheel's acceleration does not
            // depend much on s_ddot. But the curve term still matters.
            //
            // If the curve term alone exceeds the wheel acceleration limit,
            // then the current s_dot is already infeasible. Usually this
            // should have been prevented by the precomputed s_dot ceiling.
            if (a_wheel_curve_term > a_wheel_max_pos ||
                a_wheel_curve_term < a_wheel_max_neg) {
                limits.max_s_ddot_accel = 0.0f;
                limits.max_s_ddot_decel = 0.0f;
            }
        }
    }

    // ------------------------------------------------------------
    // 2. Traction limit, converted to path acceleration
    // ------------------------------------------------------------
    float path_speed_ratio =
        std::sqrt(dx_ds * dx_ds + dy_ds * dy_ds);

    if (path_speed_ratio > EPS) {
        // True translational acceleration from the path second derivative:
        //
        // ax = d2x_ds2 * s_dot^2 + dx_ds * s_ddot
        // ay = d2y_ds2 * s_dot^2 + dy_ds * s_ddot
        //
        // For the traction bound, approximate the part already required
        // for following the curve as centripetal acceleration.
        float current_v_sq =
            current_state.vx * current_state.vx
          + current_state.vy * current_state.vy;

        float a_lateral_used =
            current_v_sq * std::abs(kappa);

        a_lateral_used =
            std::min(a_lateral_used, a_max_grip_accel);

        float a_tangential_avail =
            std::sqrt(
                a_max_grip_accel * a_max_grip_accel
              - a_lateral_used * a_lateral_used
            );

        float s_ddot_traction =
            a_tangential_avail / path_speed_ratio;

        limits.max_s_ddot_accel =
            std::min(limits.max_s_ddot_accel,
                     s_ddot_traction);

        limits.max_s_ddot_decel =
            std::min(limits.max_s_ddot_decel,
                     s_ddot_traction);
    }

    return limits;
}
```

### 4. Action Chunk Discretization

```cpp
struct GlobalAction {
    float vx_global, vy_global, omega;
    float ax_global, ay_global, alpha;
};

struct ActionChunk {
    uint64_t trajectory_id;
    uint64_t start_time_pi;      // Absolute Pi execution timestamp
    uint16_t dt_ms;              // e.g., 2ms steps
    uint16_t num_actions;
    GlobalAction actions[60];
};
```

The continuous, smooth velocity and acceleration profiles resulting from the S-curve solver are sliced into fixed time steps ($dt = 2\text{ ms}$ or $4\text{ ms}$). These time slices are packed directly into the `GlobalAction` fields inside the `ActionChunk` array buffer:

```cpp
// S-curve profile generator loop packing global targets for the chunk
for (int i = 0; i < num_actions; i++) {
    float t_slice = i * (dt_ms / 1000.0f);

    // Evaluate the S-curve state vectors at this timestamp
    chunk.actions[i].vx_global = sample_s_curve_velocity_x(t_slice);
    chunk.actions[i].vy_global = sample_s_curve_velocity_y(t_slice);
    chunk.actions[i].omega     = sample_s_curve_omega(t_slice);

    // Feedforward acceleration terms strictly match the current S-curve derivative
    chunk.actions[i].ax_global = sample_s_curve_accel_x(t_slice);
    chunk.actions[i].ay_global = sample_s_curve_accel_y(t_slice);
    chunk.actions[i].alpha     = sample_s_curve_alpha(t_slice);
}
```

### **Step 5: Asynchronous Clock Synchronization (Background)**

Maintained via ping/pong serial packets on the Teensy/Pi interface:

$\text{Latency} = \frac{t_1 - t_0}{2}$

$\text{clock\_offset} = t_{\text{pi}} - (t_1 - \text{Latency})$

#### Step 6: Non-Blocking Trajectory Buffer

*Runs asynchronously as fast as the serial hardware registers populate.*

When a new `ActionChunk` packet arrives on the Teensy:

- Read `packet.trajectory_id`.

$t_{\text{start\_teensy}} = \text{packet.start\_time\_pi} - \text{clock\_offset}$

### **Step 7: Action Indexing, Frame Rotation & Coriolis Compensation (Teensy Loop)**

**Handing Early Action Chunk**

1. Teensy maintains two separate memory spaces: an **Active Buffer** (what the wheels are tracking right now) and a **Queued Buffer** (the next incoming plan).
2. When pi computes new trajectory it populates this queued buffer
3. Every cycle, the Teensy compares its current clock time against the queued chunk’s absolute execution timestamp (mapped into Teensy time via the clock synchronization offset).
4. When the timeline matches, the execution loop performs a hot pointer swap. The queued chunk instantly becomes the active chunk. This means early chunks seamlessly pick up the trajectory at the exact millisecond the old path terminates,

**Handling Late Chunk**

1. teensy simply hot swaps to the new chunk and starts at the appropriate execution time step in the middle of the new chunk

```cpp
// Persistent buffers
ActionChunk active_chunk;
ActionChunk queued_chunk;

uint64_t t_start_active_teensy = 0;
uint64_t t_start_queued_teensy = 0;

volatile bool has_queued_chunk = false;
bool is_first_chunk_of_match = true;

/**
 *Call back to teensy serial interrupt*/
void onNewActionChunkReceived(ActionChunk packet) {
    if (packet.trajectory_id <= active_chunk.trajectory_id && !is_first_chunk_of_match) {
        return; // Ignore stale or duplicate out-of-order packets
    }
    queued_chunk = packet;
    t_start_queued_teensy = packet.start_time_pi - clock_offset; 
    has_queued_chunk = true;
}

void loop() {

    uint64_t t_local = micros();

    if (has_queued_chunk) {
        if (t_local >= t_start_queued_teensy || is_first_chunk_of_match) {
       // The new chunk is late/current
            active_chunk = queued_chunk;
            t_start_active_teensy = t_start_queued_teensy;
            has_queued_chunk = false;
            is_first_chunk_of_match = false;
        }
        else{
        //The new chunk is early so keep it in the queue
        }
    }

    // Initialization block: Wait until the system receives its first valid packet
    if (is_first_chunk_of_match || t_local < t_start_active_teensy) {
        executeAsymmetricDrive(0, 0, 0, 0, 0, 0); // Active brake safety lock
        return;
    }

//Choosing the closest discrete action based on real execution time
    uint64_t elapsed_us = t_local - t_start_active_teensy;
    uint32_t action_index = (elapsed_us / 1000) / active_chunk.dt_ms;

    GlobalAction target;

    if (action_index >= active_chunk.num_actions) {
    //Pi stopped sending new action chunks and current action chunk is fully executed
        uint32_t grace_index_limit = active_chunk.num_actions + (20 / active_chunk.dt_ms);
        
        if (action_index < grace_index_limit) {
            // Hold the last target speeds, but zero out feedforward forces to stabilize
            target = active_chunk.actions[active_chunk.num_actions - 1];
            target.ax_global = 0.0f;
            target.ay_global = 0.0f;
            target.alpha     = 0.0f;
        } else {
            emergency_brake(); 
            return; 
        }
    } else {
        //nominal control flow (what is supposed to happen)
        target = active_chunk.actions[action_index];
    }

    
    // compass data must follow this convention (0 = North, 90 = East)
    float theta_actual = read_imu_absolute_heading(); 
    float vx_actual    = read_mouse_x_velocity();     
    float vy_actual    = read_mouse_y_velocity();     
    float omega_actual = read_gyro_z_rate();          

    // 1. GLOBAL-TO-LOCAL ROTATION 
    float cos_th = cos(theta_actual);
    float sin_th = sin(theta_actual);

    float vx_local_target =  target.vx_global * cos_th - target.vy_global * sin_th;
    float vy_local_target =  target.vx_global * sin_th + target.vy_global * cos_th;
    
    float ax_abs_local    =  target.ax_global * cos_th - target.ay_global * sin_th;
    float ay_abs_local    =  target.ax_global * sin_th + target.ay_global * cos_th;

    // 2. CORIOLIS FEEDFORWARD COMPENSATION (Clockwise-Positive Frame)
    // I have no idea wtf this is for icl
    float ax_local_total = ax_abs_local - omega_actual * vy_actual;
    float ay_local_total = ay_abs_local + omega_actual * vx_actual;

    // 3. Mouse sensor/compass PID
    float vx_corr = vx_local_target + Kp_x * (vx_local_target - vx_actual);
    float vy_corr = vy_local_target + Kp_y * (vy_local_target - vy_actual);
    float omega_corr = target.omega + Kp_omega * (target.omega - omega_actual);

    executeAsymmetricDrive(vx_corr, vy_corr, omega_corr, ax_local_total, ay_local_total, target.alpha);
}
```

### **Step 8: Asymmetric Kinematics & Proportional Voltage Safeguard (Teensy Loop)**

Maps corrected chassis behavior directly to asymmetric wheel coordinates. It incorporates the `soft_sign` function to eliminate high-frequency motor chattering near zero velocity, and scales voltage to prevent geometry distortion under battery sag.

```cpp
// Helper function to smooth out the Coulomb friction deadband
float soft_sign(float w, float epsilon = 0.2f){
    if (abs(w) < epsilon) {
        return w / epsilon; // Linear ramp down to 0
    }
    return (w > 0) ? 1.0f : -1.0f;
}

void executeAsymmetricDrive(float vx, float vy, float omega, float ax, float ay, float alpha){
    //PLEASE DOUBLE CHECK ANGLES I JUST GUESSED 40degree AND AM TOO LAZY TO CONFIRM
    const float alpha_wheel[4] = {-2.2689, -0.8726, 0.8726, 2.2689}; 
    const float R[4] = {0.09, 0.09, 0.09, 0.09};
    const float r_wheel = 0.025;

    float target_voltages[4];
    float max_req_voltage = 0.0f;

    for (int i = 0; i < 4; i++) {
       //velocity/accel projections to local coordinate system
        float v_wheel = sin(alpha_wheel[i]) * vx + cos(alpha_wheel[i]) * vy - R[i] * omega;
        float a_wheel = sin(alpha_wheel[i]) * ax + cos(alpha_wheel[i]) * ay - R[i] * alpha;

        float w_motor = v_wheel / r_wheel;
        float alpha_motor = a_wheel / r_wheel;

        //Motor feed forward model (some cool open loop control theory stuff)
        target_voltages[i] = kS * soft_sign(w_motor) + kV * w_motor + kA * alpha_motor;

        if (abs(target_voltages[i]) > max_req_voltage) {
            max_req_voltage = abs(target_voltages[i]);
        }
    }

    // 3. Voltage Scaling Protection
    float v_bus = read_battery_rail_voltage();
    if (max_req_voltage > v_bus) {
        float scaling = v_bus / max_req_voltage;
        for (int i = 0; i < 4; i++) {
            target_voltages[i] *= scaling;
        }
    }

    // 4. Output Mapping
    for (int i = 0; i < 4; i++) {
        set_motor_pwm_channel(i, convert_voltage_to_pwm(target_voltages[i], v_bus));
    }
}
```

## Defense Pose Computation:

#### Step 1: Compute time to Intercept

$P_g$= coord of goal

$P_b$= coord of ball

$P_r$ = coord of robot

1. $\hat{u}_{\text{ball-goal}} = \frac{P_b - P_g}{\|P_b - P_g\|}$
2. Compute Coordinate on Goal Line
    1. $m = \frac{y_b}{|x_b|}$
    
    ```cpp
    if (m>= 1) {
    x_t = 40/m;
    y_t = 40;
    }
    else if (m <= 5/11){
    	x_t = 55
    	y_t = 55*m
    }
    else{
    	A = 1 + m^2
    	B = -2(40 + 25m)
    	C = 2000
    	x_t = (-B + \sqrt(B^2 - 4AC))/(2A)
    	y_t = mx_t
    }
    x_t = sign(x_b)*x_t
    ```
    
3. $\theta_{target} = \text{atan2}(\hat{u}_{\text{ball-goal}, x}, \hat{u}_{\text{ball-goal}, y})$
4. Initial target $P_\text{initial}$ = $[x_t,y_t, \theta_\text{target}]$
5. Run a low resolution version of A* algo to find the approximate time $\tau$ to get to the target pose
6.  $d_{\text{score}} = \sqrt{(x_b - x_t)^2 + (y_b - y_t)^2}$
7. $V_{\text{approach}} = -(\mathbf{v}_b \cdot \hat{u}_{\text{ball-goal}})$
    1. if V_approach is negative then just do normal transition to block pose
8. $d_{\max} = V_{\text{approach}} \cdot \Delta t_{\text{cam}} \left( \frac{1}{1 - \gamma} \right)$
    1. if $d_\text{max}$ < $d_\text{score}$ then do normal transition to block pose
9. $N = \frac{\ln\left(1 - \frac{d_{\text{score}}(1 - \gamma)}{V_{\text{approach}} \cdot \Delta t_{\text{cam}}}\right)}{\ln(\gamma)}$
10. $T_{\text{impact}} = N \cdot \Delta t_{\text{cam}}$
    1. if $T_{\text{impact}} > \tau + \text{margin}$ do normal transition to block pose
    2. else:
        1. override target pose to have $V_\text{max}$ pointing perpendicular to the from goal line and toward the interception point
        2. you must compute V_max by:
            1. Run real A* with initial target
            2. $\phi_{\text{approach\_global}} = \text{atan2}(y_t - y_{t-1}, x_t - x_{t-1})$ (last and penultimate nodes of A*)
            3. $\phi_{\text{local\_end}} = \phi_{\text{approach\_global}} - \theta_{t-1}$ (penultimate heading from A*)
            4. using asymmetric velocity max formulas set V_max for the pose
            5. angular velocity target stays 0

1. Find future ball state
    1. $\tau_{\text{safe}} = \min(\tau, \ 0.5 \text{ seconds})$
    2. $N_\tau = \tau_{\text{safe}} \cdot \text{fps}_{\text{cam}}$
    3. $x_{b, \text{future}} = x_b + v_{bx} \cdot \Delta t_{\text{cam}} \left( \frac{1 - \gamma^{N_\tau}}{1 - \gamma} \right)$
    4. $y_{b, \text{future}} = y_b + v_{by} \cdot \Delta t_{\text{cam}} \left( \frac{1 - \gamma^{N_\tau}}{1 - \gamma} \right)$
    5. $v_{bx,\text{future}} = v_{bx} \cdot \gamma^{N_\tau}\\v_{by,\text{future}} = v_{by} \cdot \gamma^{N_\tau}$
    6. rerun these new points through the goal line point finder (code block above)
        1. use these new coords as target pos
2. Computing target velocity (use future ball velocities):
    1. $\omega_{\text{ray}} = \frac{v_{bx,\text{future}}(Y_{b,\text{future}} - Y_g) - v_{by,\text{future}}(X_{b,\text{future}} - X_g)}{\|P_{b,\text{future}} - P_g\|^2}$
    2. $\mathbf{r} = P_{\text{target}} - P_g$
    3. $\hat{\mathbf{u}}_{\text{ray}} = \frac{\mathbf{r}}{\|\mathbf{r}\|}$
    4. $\hat{\mathbf{u}}_{\text{path}}$ - unit tangent vector to boundary line at target
    5. $V_{\text{scalar}} = \frac{\omega_{\text{ray}} \cdot \|\mathbf{r}\|}{(\hat{\mathbf{u}}_{\text{ray}, y} \cdot \hat{\mathbf{u}}_{\text{path}, x}) - (\hat{\mathbf{u}}_{\text{ray}, x} \cdot \hat{\mathbf{u}}_{\text{path}, y})}$
    6. $\mathbf{V}_{\text{target}} = \begin{bmatrix} V_{\text{scalar}} \cdot \hat{\mathbf{u}}_{\text{path}, x} \\ V_{\text{scalar}} \cdot \hat{\mathbf{u}}_{\text{path}, y} \end{bmatrix}$ (using the Universal Pacing Equation)
    7. $\mathbf{d}_{\text{rel}} = \begin{bmatrix} x_{b, \text{future}} - x_{\text{target}} \\ y_{b, \text{future}} - y_{\text{target}} \end{bmatrix}$
    8. $\mathbf{v}_{\text{rel}} = \begin{bmatrix} v_{bx, \text{future}} - V_{\text{target}, x} \\ v_{by, \text{future}} - V_{\text{target}, y} \end{bmatrix}$
    9. $\omega_{\text{target}} = \frac{d_{\text{rel}, y} \cdot v_{\text{rel}, x} - d_{\text{rel}, x} \cdot v_{\text{rel}, y}}{\|\mathbf{d}_{\text{rel}}\|^2}$
    10. Clamp this velocity target by max robot velocity and max angular velocities
        1. find using same way as for emergency value