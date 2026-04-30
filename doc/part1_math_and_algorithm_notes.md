# Part 1 Nonholonomic A* Planner: Algorithm and Math Notes

這份筆記整理 `references/ENPM661_Group4_Project3/Part01` 目前在做的事情，重點放在演算法流程、狀態/動作表示、motion primitive、collision checking、cost/search，以及程式中實際用到的數學公式。

## 1. Part 1 的整體目標

Part 1 是一個純 Python 的 2D nonholonomic A* planner。

它做的事情是：

1. 建立一個 2D workspace map。
2. 載入或設定 robot motion/collision profile。
3. 用 `(x, y, theta)` 作為 planner state。
4. 用離散 motion primitives 產生 successor。
5. 對每個 primitive 的 sampled trajectory 做 static obstacle collision checking。
6. 用 A* 搜尋 start pose 到 goal region 的可行路徑。
7. 輸出 path、explored tree、summary JSON、planner log 和 visualization。

目前 Part 1 不處理 dynamic obstacles，也不處理 time-dependent edge validity。時間只存在於 motion integration 的 primitive duration 裡，沒有進入 A* search state。

## 2. Coordinate Frame and Workspace

Course frame:

- `+x`: 從 start side 指向 finish line。
- `+y`: 橫跨 course 的方向。
- world size: `4.0 m x 2.0 m`。

World bounds:

$$
x \in [0, W], \qquad y \in [0, H]
$$

其中：

$$
W = 4.0 \text{ m}, \qquad H = 2.0 \text{ m}
$$

程式中 bounds 表示為：

$$
(x_{\min}, x_{\max}, y_{\min}, y_{\max}) = (0, W, 0, H)
$$

## 3. State Representation

Planner 的 continuous state 是：

$$
\mathbf{x} = (x, y, \theta)
$$

其中：

- `x`, `y`: robot reference point 在 world frame 的位置。
- `theta`: robot heading，單位 radians。

對 team car 而言，這個 reference point 是 measured wheel-center reference point，不完全等於 raw `base_link` origin。

## 4. Angle Normalization

程式將 heading normalization 到：

$$
\theta \in [-\pi, \pi)
$$

公式：

$$
\operatorname{normalize}(\theta)
= ((\theta + \pi) \bmod 2\pi) - \pi
$$

這用在：

- primitive rollout 更新 heading。
- goal heading error。
- heading discretization。
- local pose delta。

## 5. State Discretization

A* 的 closed set / best cost dictionary 使用離散 key：

$$
k = (i_x, i_y, i_\theta)
$$

位置離散化：

$$
i_x = \operatorname{round}\left(\frac{x}{r_{xy}}\right)
$$

$$
i_y = \operatorname{round}\left(\frac{y}{r_{xy}}\right)
$$

其中 `r_xy = xy_resolution_m`，預設：

$$
r_{xy} = 0.05 \text{ m}
$$

heading bin：

$$
i_\theta
= \left\lfloor
\frac{\operatorname{normalize}(\theta) + \pi}{2\pi / N_\theta}
\right\rfloor
\bmod N_\theta
$$

預設：

$$
N_\theta = 24
$$

所以每個 heading bin 大約是：

$$
\Delta \theta = \frac{2\pi}{24} = 15^\circ
$$

## 6. Goal Condition

Goal 是一個 `(x_g, y_g)` target，加上固定 desired heading。

位置條件：

$$
\sqrt{(x - x_g)^2 + (y - y_g)^2} \le r_g
$$

預設：

$$
r_g = 0.15 \text{ m}
$$

heading 條件：

$$
|\operatorname{normalize}(\theta - \theta_g)| \le \epsilon_\theta
$$

預設：

$$
\theta_g = 0
$$

$$
\epsilon_\theta = 15^\circ
$$

因此 goal satisfied 需要同時滿足 position tolerance 和 heading tolerance。

## 7. Euclidean Distance

程式中最基本的距離公式：

$$
d((x_1,y_1),(x_2,y_2))
= \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2}
$$

這用於：

- A* heuristic。
- goal checking。
- primitive arc-length cost 累積。

## 8. Robot Motion Profiles

Part 1 支援兩種 motion model：

1. `team_car`: rear-drive-steered / kinematic bicycle approximation。
2. `turtlebot`: differential-drive。

### 8.1 RPM to Angular Speed

RPM 轉 rad/s：

$$
\omega_{\text{wheel}}
= \text{RPM} \cdot \frac{2\pi}{60}
$$

程式函式：

```python
rpm_to_rad_per_sec(rpm)
```

## 9. Differential-Drive Motion Model

TurtleBot profile 使用 differential-drive model。

左右輪角速度：

$$
\omega_L = \text{RPM}_L \cdot \frac{2\pi}{60}
$$

$$
\omega_R = \text{RPM}_R \cdot \frac{2\pi}{60}
$$

線速度：

$$
v = r \frac{\omega_L + \omega_R}{2}
$$

角速度：

$$
\omega = r \frac{\omega_R - \omega_L}{b}
$$

其中：

- `r = wheel_radius_m`
- `b = track_width_m`

TurtleBot default:

$$
r = 0.033 \text{ m}
$$

$$
b = 0.160 \text{ m}
$$

## 10. Rear-Drive-Steered / Bicycle Motion Model

Team car profile 使用 rear-drive-steered kinematic approximation。

後輪角速度：

$$
\omega_{\text{rear}} = \text{RPM}_{\text{drive}} \cdot \frac{2\pi}{60}
$$

線速度：

$$
v = r \omega_{\text{rear}}
$$

steering angle:

$$
\delta = \delta_{\text{deg}} \cdot \frac{\pi}{180}
$$

若 steering angle 接近 0：

$$
\omega = 0
$$

否則：

$$
\omega = \frac{v \tan(\delta)}{L}
$$

其中：

- `r = wheel_radius_m`
- `L = wheelbase_m`

Team car default:

$$
r = 0.028042 \text{ m}
$$

$$
L = 0.140208 \text{ m}
$$

$$
b = 0.1367024 \text{ m}
$$

Team car 還有一個 reference point offset：

$$
d_{\text{rear}\to\text{ref}} = 0.070104 \text{ m}
$$

這代表 planner state reference point 不在 rear axle center，而是往前偏移。

## 11. Continuous-Time Pose Integration

每個 action primitive 被 rollout 成一串 sampled poses。

給定目前 pose：

$$
(x_t, y_t, \theta_t)
$$

primitive 在小時間步長 `dt` 下更新。

程式使用 midpoint heading：

$$
\theta_{\text{mid}} = \theta_t + \frac{\omega \Delta t}{2}
$$

### 11.1 Differential-Drive Case

若 `rear_to_reference = 0`，更新式等價於：

$$
x_{t+\Delta t}
= x_t + v \cos(\theta_{\text{mid}})\Delta t
$$

$$
y_{t+\Delta t}
= y_t + v \sin(\theta_{\text{mid}})\Delta t
$$

$$
\theta_{t+\Delta t}
= \operatorname{normalize}(\theta_t + \omega \Delta t)
$$

### 11.2 Team-Car Reference-Point Case

如果 reference point 與 rear axle 有距離 `d = rear_to_reference_m`，程式使用：

$$
x_{t+\Delta t}
= x_t
+ \left(
v\cos(\theta_{\text{mid}})
- d\omega\sin(\theta_{\text{mid}})
\right)\Delta t
$$

$$
y_{t+\Delta t}
= y_t
+ \left(
v\sin(\theta_{\text{mid}})
+ d\omega\cos(\theta_{\text{mid}})
\right)\Delta t
$$

$$
\theta_{t+\Delta t}
= \operatorname{normalize}(\theta_t + \omega \Delta t)
$$

其中：

$$
d = d_{\text{rear}\to\text{ref}}
$$

## 12. Primitive Duration and Integration Resolution

每個 primitive 持續固定時間：

$$
T = \text{action\_duration\_s}
$$

預設：

$$
T = 1.0 \text{ s}
$$

integration step:

$$
\Delta t = \text{integration\_dt\_s}
$$

預設：

$$
\Delta t = 0.05 \text{ s}
$$

每個 primitive 會產生：

$$
N = \left\lceil \frac{T}{\Delta t} \right\rceil
$$

個 integration intervals，samples 包含 start pose 和每個 step 的新 pose。

## 13. Motion Primitive Action Set

### 13.1 Team Car Action Set

Team car 使用 semantic primitives：

```text
straight_fast
straight_slow
right
right_fast
left
left_fast
```

對應 action set：

$$
(\text{RPM}_2, 0^\circ)
$$

$$
(\text{RPM}_1, 0^\circ)
$$

$$
(\text{RPM}_1, -\delta_{\text{normal}})
$$

$$
(\text{RPM}_2, -\delta_{\text{fast}})
$$

$$
(\text{RPM}_1, +\delta_{\text{normal}})
$$

$$
(\text{RPM}_2, +\delta_{\text{fast}})
$$

預設：

$$
\delta_{\text{normal}} = 30^\circ
$$

$$
\delta_{\text{fast}} = 20^\circ
$$

常見 run command 使用：

$$
\text{RPM}_1 = 20, \qquad \text{RPM}_2 = 40
$$

### 13.2 Differential-Drive Action Set

TurtleBot action set：

$$
(0, \text{RPM}_1)
$$

$$
(\text{RPM}_1, 0)
$$

$$
(\text{RPM}_1, \text{RPM}_1)
$$

$$
(0, \text{RPM}_2)
$$

$$
(\text{RPM}_2, 0)
$$

$$
(\text{RPM}_2, \text{RPM}_2)
$$

$$
(\text{RPM}_1, \text{RPM}_2)
$$

$$
(\text{RPM}_2, \text{RPM}_1)
$$

## 14. Calibrated Team-Car Primitives

Team car action 若有 `action_name`，程式會優先使用 embedded calibrated primitive table，而不是直接用 bicycle model integration。

每個 primitive 有 local displacement：

$$
(\Delta x_{\text{local}}, \Delta y_{\text{local}}, \Delta\theta)
$$

如果 query 的 action duration 與 calibration duration 不同，使用 duration scale：

$$
s = \frac{T}{T_{\text{calib}}}
$$

scaled displacement:

$$
\Delta x = s \Delta x_{\text{calib}}
$$

$$
\Delta y = s \Delta y_{\text{calib}}
$$

$$
\Delta \theta = s \Delta\theta_{\text{calib}}
$$

對第 `i` 個 sample：

$$
\alpha_i = \frac{i}{N}
$$

local interpolation:

$$
x_{\text{local},i} = \alpha_i \Delta x
$$

$$
y_{\text{local},i} = \alpha_i \Delta y
$$

$$
\theta_i = \theta_0 + \alpha_i \Delta\theta
$$

轉到 world frame：

$$
x_i
= x_0
+ x_{\text{local},i}\cos\theta_0
- y_{\text{local},i}\sin\theta_0
$$

$$
y_i
= y_0
+ x_{\text{local},i}\sin\theta_0
+ y_{\text{local},i}\cos\theta_0
$$

$$
\theta_i
= \operatorname{normalize}(\theta_0 + \alpha_i\Delta\theta)
$$

目前 embedded table 中每個 primitive 都有：

- mean delta x
- mean delta y
- mean delta theta
- standard deviation of delta x/y/theta
- number of trials
- failure count

## 15. Primitive Cost

每個 primitive 的 travel cost 是 sampled trajectory 的 arc length approximation：

$$
c_{\text{seg}}
= \sum_{i=1}^{N}
\sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2}
$$

如果 segment cost 幾乎為 0：

$$
c_{\text{seg}} \le 10^{-12}
$$

則該 successor 被忽略。

## 16. A* Search

### 16.1 Node Stored in Search

每個 A* node 儲存：

- continuous pose `(x, y, theta)`
- parent key
- primitive segment from parent
- action
- travel cost to come
- uncertainty/risk cost to come
- total cost to come
- covariance and uncertainty margin

### 16.2 Cost-to-Come

travel cost:

$$
g_{\text{travel}}(n')
= g_{\text{travel}}(n) + c_{\text{seg}}
$$

若 uncertainty-aware planning enabled，還有 risk cost：

$$
g_{\text{risk}}(n')
= g_{\text{risk}}(n)
+ w_{\text{risk}} m_{\text{uncertainty}}
$$

total cost:

$$
g(n') = g_{\text{travel}}(n') + g_{\text{risk}}(n')
$$

如果沒有 uncertainty-aware planning：

$$
g(n') = g_{\text{travel}}(n')
$$

### 16.3 Heuristic

A* heuristic 是 Euclidean distance 到 goal：

$$
h(n)
= \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}
$$

priority:

$$
f(n) = g(n) + h(n)
$$

程式的 heap item 大致是：

$$
(f, h, \text{tie\_breaker}, \text{state\_key})
$$

`h` 被放進 heap 是為了 tie-breaking。

### 16.4 Best-Cost Update

對 successor key `k'`，如果：

$$
g_{\text{new}} \ge g_{\text{best}}(k')
$$

則不更新。

否則：

$$
g_{\text{best}}(k') \leftarrow g_{\text{new}}
$$

並把 successor push 到 open heap。

### 16.5 Termination

搜尋停止條件：

1. 找到 goal-satisfied node。
2. open list exhausted。
3. expanded node 數達到：

$$
N_{\max} = \text{max\_iterations}
$$

預設：

$$
N_{\max} = 50000
$$

## 17. Collision Model

Part 1 支援兩種 robot footprint：

1. circle footprint
2. oriented box footprint

Static obstacles 都是 oriented rectangles。

## 18. Circle Robot Bounds Check

如果 robot 用 circle footprint，有效半徑：

$$
r_{\text{eff}} = r_{\text{robot}} + c
$$

其中 `c = clearance`。

world bounds valid condition：

$$
x_{\min} + r_{\text{eff}}
\le x
\le x_{\max} - r_{\text{eff}}
$$

$$
y_{\min} + r_{\text{eff}}
\le y
\le y_{\max} - r_{\text{eff}}
$$

## 19. Circle vs Oriented Box Collision

給定 circle center `(x, y)` 和 oriented box：

$$
B = (x_b, y_b, w, h, \phi)
$$

先把 circle center 轉到 box local frame。

world offset:

$$
d_x = x - x_b
$$

$$
d_y = y - y_b
$$

旋轉 by `-phi`：

$$
x_{\text{local}}
= d_x\cos(-\phi) - d_y\sin(-\phi)
$$

$$
y_{\text{local}}
= d_x\sin(-\phi) + d_y\cos(-\phi)
$$

box half extents：

$$
a = \frac{w}{2}
$$

$$
b = \frac{h}{2}
$$

local closest point：

$$
x_c = \min(\max(x_{\text{local}}, -a), a)
$$

$$
y_c = \min(\max(y_{\text{local}}, -b), b)
$$

collision condition：

$$
\sqrt{(x_{\text{local}} - x_c)^2 + (y_{\text{local}} - y_c)^2}
\le r_{\text{eff}}
$$

如果任一 obstacle collision，pose invalid。

## 20. Oriented Box Robot Footprint

若 robot 用 box footprint，先從 pose 建立 robot oriented box。

原本 robot footprint:

$$
l = \text{length\_m}
$$

$$
w = \text{width\_m}
$$

clearance expansion:

$$
l_{\text{eff}} = l + 2c
$$

$$
w_{\text{eff}} = w + 2c
$$

如果 collision profile 有 center offset：

$$
(o_x, o_y)
$$

則 footprint center:

$$
x_c = x + o_x\cos\theta - o_y\sin\theta
$$

$$
y_c = y + o_x\sin\theta + o_y\cos\theta
$$

robot box angle:

$$
\phi = \theta
$$

## 21. Oriented Box Corners

對任一 oriented box：

$$
B = (x_c, y_c, w, h, \phi)
$$

local corners:

$$
(-w/2,-h/2), (w/2,-h/2), (w/2,h/2), (-w/2,h/2)
$$

轉到 world:

$$
x = x_c + x_{\ell}\cos\phi - y_{\ell}\sin\phi
$$

$$
y = y_c + x_{\ell}\sin\phi + y_{\ell}\cos\phi
$$

## 22. Oriented Box vs Oriented Box Collision

程式用 separating axis theorem。

兩個 boxes `A` 和 `B` 的 candidate axes：

$$
\mathbf{a}_1 = (\cos\phi_A, \sin\phi_A)
$$

$$
\mathbf{a}_2 = (-\sin\phi_A, \cos\phi_A)
$$

$$
\mathbf{b}_1 = (\cos\phi_B, \sin\phi_B)
$$

$$
\mathbf{b}_2 = (-\sin\phi_B, \cos\phi_B)
$$

對每個 axis `u`，將 box corners project 到 axis：

$$
p_i = \mathbf{c}_i \cdot \mathbf{u}
$$

projection interval：

$$
I = [\min_i p_i, \max_i p_i]
$$

若存在某個 axis 讓 intervals 不重疊：

$$
\max I_A < \min I_B
\quad \text{or} \quad
\max I_B < \min I_A
$$

則兩 boxes 不相交。

如果所有 candidate axes 都沒有 separating axis，則 boxes intersect。

## 23. Trajectory Collision Checking

primitive collision-free 的條件是所有 sampled poses 都 valid：

$$
\forall i \in \{0,\ldots,N\},
\quad
\operatorname{valid}(x_i, y_i, \theta_i) = \text{true}
$$

也就是：

$$
\operatorname{collisionFree}(\tau)
= \bigwedge_{i=0}^{N}
\operatorname{poseValid}(\mathbf{x}_i)
$$

目前只檢查 static obstacles 和 world bounds。

## 24. Uncertainty-Aware Planning

Part 1 對 team car 有一個 optional uncertainty-aware extension。這不是一般 A* 必需項，但目前程式預設對 team car 開啟。

### 24.1 Local Pose Delta

給定 start pose 和 end pose：

world displacement：

$$
\Delta x_w = x_e - x_s
$$

$$
\Delta y_w = y_e - y_s
$$

轉到 start local frame：

$$
\Delta x_{\ell}
= \Delta x_w\cos\theta_s + \Delta y_w\sin\theta_s
$$

$$
\Delta y_{\ell}
= -\Delta x_w\sin\theta_s + \Delta y_w\cos\theta_s
$$

heading delta：

$$
\Delta\theta
= \operatorname{normalize}(\theta_e - \theta_s)
$$

### 24.2 Covariance Propagation

current covariance：

$$
\Sigma_k
$$

Jacobian：

$$
J =
\begin{bmatrix}
1 & 0 & -\sin\theta \Delta x_{\ell} - \cos\theta \Delta y_{\ell} \\
0 & 1 & \cos\theta \Delta x_{\ell} - \sin\theta \Delta y_{\ell} \\
0 & 0 & 1
\end{bmatrix}
$$

world rotation:

$$
R =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

local primitive process noise:

$$
Q_{\ell}
=
\begin{bmatrix}
\sigma_x^2 & 0 & 0 \\
0 & \sigma_y^2 & 0 \\
0 & 0 & \sigma_{\theta}^2
\end{bmatrix}
$$

propagated covariance:

$$
\Sigma_{k+1}
= J \Sigma_k J^T + R Q_{\ell} R^T
$$

### 24.3 Uncertainty Metrics

取 position covariance：

$$
\Sigma_{xy} = \Sigma[0:2,0:2]
$$

largest eigenvalue：

$$
\lambda_{\max}
= \max(\operatorname{eig}(\Sigma_{xy}))
$$

position uncertainty:

$$
\sigma_{xy}
= \sqrt{\max(\lambda_{\max}, 0)}
$$

heading uncertainty:

$$
\sigma_{\theta}
= \sqrt{\max(\Sigma_{\theta\theta}, 0)}
$$

uncertainty margin:

$$
m_{\text{uncertainty}}
= s_{\sigma}
\left(
\sigma_{xy}
+ s_{\theta} r_{\text{robot}}\sigma_{\theta}
\right)
$$

where:

$$
s_{\sigma} = \text{uncertainty\_sigma\_scale}
$$

$$
s_{\theta} = \text{uncertainty\_heading\_radius\_scale}
$$

預設：

$$
s_{\sigma} = 0.25
$$

$$
s_{\theta} = 0.35
$$

### 24.4 Collision Inflation with Uncertainty

primitive collision checking 使用：

$$
c_{\text{effective}}
= c_{\text{clearance}} + m_{\text{uncertainty}}
$$

也就是 uncertainty 越大，collision checking 越保守。

### 24.5 Risk Cost

risk cost increment:

$$
\Delta g_{\text{risk}}
= w_{\text{risk}} m_{\text{uncertainty}}
$$

預設：

$$
w_{\text{risk}} = 0.15
$$

## 25. Map Obstacle Geometry

Internal obstacles 用 oriented rectangles 表示：

$$
B_i = (x_i, y_i, w_i, h_i, \phi_i)
$$

目前 map 包含：

- `bottom_left_bar`
- `top_bar`
- `bottom_right_bar`
- `square_upper_left`
- `square_mid_left`
- `square_lower_right`

Course-frame obstacle values already expressed in meters, for example：

```text
bottom_left_bar center = (0.7483, 1.3813)
top_bar center         = (2.95, 1.2750)
bottom_right_bar       = (1.6113, 0.5971)
```

bar thickness:

$$
t_{\text{bar}} = 0.05 \text{ m}
$$

square size:

$$
s_{\text{square}} = 0.304 \text{ m}
$$

bar angles:

$$
\phi_{\text{bottom-left}} = -60^\circ
$$

$$
\phi_{\text{top}} = 90^\circ
$$

$$
\phi_{\text{bottom-right}} = 60^\circ
$$

Older drawing-frame transform formulas still exist in the file.

Drawing coordinate to course coordinate:

$$
x_{\text{course,cm}} = y_{\text{drawing,cm}}
$$

$$
y_{\text{course,cm}} = H_{\text{world,cm}} - x_{\text{drawing,cm}}
$$

Drawing angle to course angle:

$$
\phi_{\text{course}}
= \phi_{\text{drawing}} - \frac{\pi}{2}
$$

cm to m:

$$
v_{\text{m}} = 0.01v_{\text{cm}}
$$

## 26. Vehicle Geometry Extraction

URDF parsing is mostly used for report context. The planner actually uses measured built-in team-car parameters.

Track width estimate from wheel centers:

$$
b = |\bar{y}_{\text{left}} - \bar{y}_{\text{right}}|
$$

where:

$$
\bar{y}_{\text{left}}
= \frac{1}{N_L}\sum_i y_{L,i}
$$

$$
\bar{y}_{\text{right}}
= \frac{1}{N_R}\sum_i y_{R,i}
$$

Wheelbase estimate:

$$
L = \max_i x_i - \min_i x_i
$$

STL circular collision radius estimate:

Given STL bbox size:

$$
(s_x, s_y, s_z)
$$

conservative XY radius:

$$
r = \frac{1}{2}\sqrt{s_x^2 + s_y^2}
$$

However, `team_car_circle` currently uses configured:

$$
r = 0.17 \text{ m}
$$

## 27. Main Runtime Pipeline

Part 1 `main.py` does:

1. Parse command line arguments.
2. Create `Part1Config`.
3. Resolve URDF/STL paths.
4. Build vehicle parameters.
5. Build collision parameters.
6. Build static map obstacles.
7. Convert start heading from degrees to radians:

$$
\theta_{\text{rad}}
= \theta_{\text{deg}}\frac{\pi}{180}
$$

8. Run sanity checks.
9. Run `plan(...)`.
10. Save planner log JSON.
11. Save static plot.
12. Optionally save animation.
13. Save run summary JSON.

## 28. Sanity Checks

Before planning:

1. Start inside world bounds:

$$
x_{\min}\le x_s\le x_{\max}
$$

$$
y_{\min}\le y_s\le y_{\max}
$$

2. Goal inside world bounds:

$$
x_{\min}\le x_g\le x_{\max}
$$

$$
y_{\min}\le y_g\le y_{\max}
$$

3. Clearance nonnegative:

$$
c \ge 0
$$

4. Collision radius positive:

$$
r_{\text{robot}} > 0
$$

5. Robot effective size not too large:

$$
r_{\text{robot}} + c < \frac{\min(W,H)}{3}
$$

6. Start pose collision-free.
7. Goal pose collision-free.

## 29. What Part 1 Is Not Doing Yet

Part 1 does not yet include:

1. Dynamic obstacles.
2. Explicit time in search state.
3. Edge validity intervals.
4. Roadmap construction.
5. Multi-query roadmap reuse.
6. Waiting actions.
7. Time-indexed collision checking.
8. Temporal cost or arrival-time-dependent validity.

Currently, A* state key is:

$$
(i_x, i_y, i_{\theta})
$$

For our final project, the temporal planner likely needs:

$$
(i_x, i_y, i_{\theta}, i_t)
$$

or a label-setting representation:

$$
(\text{node\_id}, t_{\text{arrival}})
$$

## 30. Natural Modification Points for Our Project

### 30.1 Add Dynamic Obstacles

A moving circular obstacle can be represented as:

$$
o_j(t) = (x_{j,0} + v_{x,j}t,\; y_{j,0} + v_{y,j}t)
$$

with radius:

$$
r_j
$$

Dynamic collision with circular robot:

$$
\sqrt{
(x_i - x_j(t_i))^2
+ (y_i - y_j(t_i))^2
}
\le r_{\text{robot}} + r_j + c
$$

### 30.2 Add Time to Primitive Samples

For a primitive starting at arrival time `t0`:

$$
t_i = t_0 + i\Delta t
$$

or if samples are interpolated over primitive duration:

$$
t_i = t_0 + \alpha_i T
$$

Then dynamic validation becomes:

$$
\operatorname{temporalCollisionFree}(\tau, t_0)
=
\bigwedge_i
\operatorname{poseValidStatic}(\mathbf{x}_i)
\land
\operatorname{poseValidDynamic}(\mathbf{x}_i, t_i)
$$

### 30.3 Convert A* to Temporal A*

Current transition:

$$
\mathbf{x}' = f(\mathbf{x}, u)
$$

Future temporal transition:

$$
(\mathbf{x}', t')
= (f(\mathbf{x}, u),\; t + T_u)
$$

where:

$$
T_u = \text{primitive duration}
$$

Cost could be:

$$
g' = g + c_{\text{seg}}
$$

or time-based:

$$
g' = g + T_u
$$

or mixed:

$$
g' = g + \alpha c_{\text{seg}} + \beta T_u
$$

### 30.4 Convert From Grid A* to Roadmap

Current Part 1 expands primitives from every reached grid state online.

For temporal roadmap:

1. Sample roadmap nodes:

$$
q_i = (x_i, y_i, \theta_i)
$$

2. Connect nodes using primitive rollouts or primitive libraries.
3. Store feasible static edges:

$$
e_{ij} = \tau_{ij}
$$

4. During query, validate edge against dynamic obstacles at arrival time:

$$
\operatorname{valid}(e_{ij}, t_i)
$$

5. Search over:

$$
(\text{node}, t)
$$

This lets us reuse Part 1's motion and collision code while changing the graph/search structure.
