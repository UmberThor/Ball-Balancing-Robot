# Ball Balancing Robot

A 3-DOF parallel platform that keeps a ball balanced in the center of a transparent plate. A camera looks at the plate from below and detects the position of the ball, a controller turns the ball position error into a desired plate tilt, and the inverse kinematics turns that tilt into three servo angles.

<p align="center">
  <img src="docs/placeholder.gif" width="640">
  <br>
  <em>The robot balancing a ball</em>
</p>

Different types of controller are tested: a PID controller, an LQR controller and a Reinforcement Learning controller.

---

# 1. Inverse Kinematics

Reference implementation: `inverse_kinematics.m`.

## 1.1 Problem statement

The plate is a rigid disc whose center is constrained to the vertical $z$ axis. It can therefore only *tilt* and *rise*, which is exactly 3 DOF.

**Input**: the pose of the plate:

- $\mathbf{n} = [\alpha,\ \beta,\ \gamma]^T$, the unit normal of the plate,
- $h$, the height of the center of the plate $\mathbf{c} = (0, 0, h)$.

**Output**: the three motor angles $q_1, q_2, q_3$.


## 1.2 Architecture

The actuation mechanism consists of three identical arms, located 120° apart. Each arm $i$ is a two-link RRS chain that connects the base to the plate, so the robot as a whole is a **3-RRS** parallel manipulator: revolute at the shoulder, revolute (pin) at the elbow, spherical at the wrist. The shoulder joint is **active**, driven by the motor; the elbow and wrist joints are **passive**.

<p align="center">
  <img src="docs/Figure_1.png" width="640">
  <br>
  <em>Architecture of the 3-DOF manipulator for <b>n</b> = [&minus;0.25, 0.33, 1]<sup>T</sup>, plotted by <code>inverse_kinematics.m</code></em>
</p>


| Point / length | Meaning |
|---|---|
| $\mathbf{M}_i$ | coordinates of the $i$-th motor, fixed on the base circle of radius $R_b$ |
| $\mathbf{P}_i$ | coordinates of the $i$-th pin joint connecting the two links |
| $\mathbf{B}_i$ | coordinates of the $i$-th ball joint on the plate circle of radius $R_p$ |
| $L_1$ | length of the upper link (link 1) that connects $\mathbf{P}_i$ to $\mathbf{B}_i$ |
| $L_2$ | length of the lower link (link 2) that connects $\mathbf{M}_i$ to $\mathbf{P}_i$ |

The physical constants are:

| Constant | Meaning | Value |
|---|---|---|
| $R_b$ | radius of the base circle | 28.65 mm |
| $R_p$ | radius of the plate circle | 84.00 mm |
| $L_1$ | length of the upper link | 80 mm |
| $L_2$ | length of the lower link | 80 mm |

## 1.3 Planes

Each arm ($\mathbf{M}_i$, $\mathbf{P}_i$ and $\mathbf{B}_i$) is confined to a fixed vertical plane. In coordinates, that plane is simply

$$y = E_i\, x .$$

where $E_i = \frac{M_{iy}}{M_{ix}}$. Under this consideration, the entire 3D problem collapses into three independent 2D problems, one per plane.

<p align="center">
  <img src="docs/Plane_1.png" alt="Plane y = 0 containing arm 1" width="32%">
  <img src="docs/Plane_2.png" alt="Plane y = -sqrt(3) x containing arm 2" width="32%">
  <img src="docs/Plane_3.png" alt="Plane y = sqrt(3) x containing arm 3" width="32%">
  <br>
  <em>The three arm planes: arm 1 is red, arm 2 is green and arm 3 is blue.<br>Whatever the pose of the plate, each arm and the ball joint it drives stay in their own plane.</em>
</p>

The coordinates of each motor are computed as the points along a circumference of radius $R_b$ spaced by 120°, so the result is:


| Arm | $\mathbf{M}_i$ | $E_i$
|-----|-------|-------|
| 1 | $(-R_b,\ 0,\ 0)$ | $0$
| 2 | $(R_b/2,\ -\tfrac{\sqrt3}{2}R_b,\ 0)$ | $-\sqrt3$ |
| 3 | $(R_b/2,\ +\tfrac{\sqrt3}{2}R_b,\ 0)$ | $+\sqrt3$ |

## 1.4 Step 1 — Ball joint coordinates $\mathbf{B}_i$

$\mathbf{B}_i$ is the intersection of three conditions:

1. it lies on the plate plane: $(\mathbf{B}_i - \mathbf{c}) \cdot \mathbf{n} = 0$
2. it lies in the arm plane
3. it lies on the plate rim

This leads to writing the system:

$$
\begin{cases}
\alpha x + \beta y + \gamma (z - h)= 0 \\
y = E_i x \\
x^2 + y^2 + (z - h)^2 = R_p^2
\end{cases}
$$

Substituting (2) into (1) gives the height along the plane as a function of $x$:

$$z = h - \frac{\alpha + \beta E_i}{\gamma}\,x$$

and substituting both into (3) leaves a pure quadratic in $x$:

$$x^2\left[\,1 + E_i^2 + \frac{(\alpha + \beta E_i)^2}{\gamma^2}\right] = R_p^2 .$$

Hence the closed form for any arm:

$$\boxed{\;
x_i = s_i\,\frac{\gamma R_p}{\sqrt{\gamma^2\left(1 + E_i^2\right) + (\alpha + \beta E_i)^2}},
\qquad y_i = E_i\,x_i,
\qquad z_i = h - \frac{\alpha + \beta E_i}{\gamma}\,x_i \;}$$

where $s_i \in \{-1, +1\}$ picks the one of the two rim points that is on the motor's side; the other root is the diametrically opposite point of the rim.

Expanding the box for the three arms gives:

**Arm 1**: $E_1 = 0$, $s_1 = -1$:

$$\mathbf{B}_1 = \left(-\frac{\gamma R_p}{\sqrt{\alpha^2 + \gamma^2}},\;\; 0,\;\;
h + \frac{\alpha R_p}{\sqrt{\alpha^2 + \gamma^2}}\right)$$

**Arm 2**: $E_2 = -\sqrt3$, $s_2 = +1$:

$$\mathbf{B}_2 = \left(
\frac{\gamma R_p}{\sqrt{4\gamma^2 + \left(\sqrt3\beta - \alpha\right)^2}},\;\;
-\frac{\sqrt3\,\gamma R_p}{\sqrt{4\gamma^2 + \left(\sqrt3\beta - \alpha\right)^2}},\;\;
h + \frac{R_p\left(\sqrt3\beta - \alpha\right)}{\sqrt{4\gamma^2 + \left(\sqrt3\beta - \alpha\right)^2}}
\right)$$

**Arm 3**: $E_3 = +\sqrt3$, $s_3 = +1$:

$$\mathbf{B}_3 = \left(
\frac{\gamma R_p}{\sqrt{4\gamma^2 + \left(\sqrt3\beta + \alpha\right)^2}},\;\;
+\frac{\sqrt3\,\gamma R_p}{\sqrt{4\gamma^2 + \left(\sqrt3\beta + \alpha\right)^2}},\;\;
h - \frac{R_p\left(\sqrt3\beta + \alpha\right)}{\sqrt{4\gamma^2 + \left(\sqrt3\beta + \alpha\right)^2}}
\right)$$


Before solving for the elbow, the two-link chain must be able to span the gap between motor and ball joint. The triangle inequality gives, per arm,

$$|L_1 - L_2| < \lVert \mathbf{B}_i - \mathbf{M}_i \rVert < L_1 + L_2$$

If such condition fails, the requested pose $(\mathbf{n}, h)$ is simply not reachable.

## 1.5 Step 2 — Pin joint coordinates $\mathbf{P}_i$

$\mathbf{P}_i$ is again the intersection of three conditions:

1. it is at distance $L_1$ from the ball joint
2. it is at distance $L_2$ from the motor
3. it lies in the arm plane

This leads to writing the system:
$$
\begin{cases}
(x - B_{ix})^2 + (y - B_{iy})^2 + (z - B_{iz})^2 = L_1^2 \\
(x - M_{ix})^2 + (y - M_{iy})^2 + (z - M_{iz})^2 = L_2^2 \\
y = E_i x
\end{cases}
$$

Subtracting sphere (2) from sphere (1) kills every quadratic term and leaves a plane, which solved for $z$ reads

$$z = A_i + B_ix + C_iy$$

with
$$A_i = \frac{L_1^2 - L_2^2 + \left(M_{ix}^2 - B_{ix}^2\right) + \left(M_{iy}^2 - B_{iy}^2\right) + \left(M_{iz}^2 - B_{iz}^2\right)}{2\,(M_{iz} - B_{iz})} $$
$$B_i = \frac{B_{ix} - M_{ix}}{M_{iz} - B_{iz}} $$
$$C_i = \frac{B_{iy} - M_{iy}}{M_{iz} - B_{iz}}$$

Geometrically this is the *radical plane* of the two spheres: the plane that contains their intersection circle.


Intersecting it with $y = E_i x$ collapses the circle to the two points of a line:

$$y = E_i x, \qquad z = A_i + (B_i + C_i E_i)\,x$$

Putting the line back into condition (1) gives

$$\underbrace{\left[1 + E_i^2 + (B_i + C_iE_i)^2\right]}_{a_i}x^2
+ \underbrace{\left[-2B_{ix} - 2E_iB_{iy} + 2(B_i + C_iE_i)(A_i - B_{iz})\right]}_{b_i}x
+ \underbrace{\left[B_{ix}^2 + B_{iy}^2 + (A_i - B_{iz})^2 - L_1^2\right]}_{c_i} = 0$$

$$x_{1,2} = \frac{-b_i \pm \sqrt{b_i^2 - 4a_ic_i}}{2a_i}$$

The two roots are the two physical assemblies of the arm: elbow folded outward and elbow folded inward. The robot is built with the elbow out, so the root to keep is the one that maximizes $|x|$: that is the $+$ root when $s_i = +1$ and the $-$ root when $s_i = -1$.

Once $x$ is found, the coordinates of the $i$-th pin joint are:

$$ \mathbf{P}_i = \Big(x,\;\; E_i x,\;\; A_i + (B_i + C_iE_i)\,x\Big)$$

## 1.6 Step 3 — motor angles $q_i$

The motor angle is the tilt of link 2 ($\mathbf{M}_i \to \mathbf{P}_i$) measured from the vertical.

$$q_i = \arctan\!\left(\frac{\sqrt{(P_{ix} - M_{ix})^2 + (P_{iy} - M_{iy})^2}}{\lvert P_{iz} - M_{iz}\rvert}\right)$$


## 1.7 Worked example

For $\mathbf{n} = [-0.25, 0.33, 1]$, $h = 100$, with the constants of §1.2:

| | $\mathbf{B}_i$ | $\mathbf{P}_i$ | $q_i$ |
|---|---|---|---|
| 1 | $(-81.49,\ 0,\ 79.63)$ | $(-108.53,\ 0,\ 4.34)$ | $86.89°$ |
| 2 | $(38.85,\ -67.29,\ 131.92)$ | $(44.42,\ -76.94,\ 52.70)$ | $48.80°$ |
| 3 | $(41.47,\ 71.82,\ 86.67)$ | $(53.97,\ 93.47,\ 10.67)$ | $82.33°$ |

## 1.8 Visualization

Running `inverse_kinematics.m` draws the whole assembly: base circle and the three motor points, plate circle with its normal arrow, the three ball joints, the elbows, and both links of every arm, color-coded red / green / blue for arms 1 / 2 / 3.

## 1.9 From Matlab model to physical robot

`inverse_kinematics.m` returns the motor angles of every reachable configuration of the robot, including the ones that need $q > 90°$. The physical robot never uses them: to balance the ball the plate must stay close to horizontal. The nominal pose is therefore $h = 120$ with $\mathbf{n} = [0, 0, 1]$, which gives $q_1 = q_2 = q_3 \approx 59°$. The commands sent to the motors are clipped to $[45°,\ 75°]$, about $\pm 15°$ around the nominal pose: enough travel to correct the position of the ball, and never enough to reach a configuration that the robot cannot assume.

# 2. Mechanical design

> **On the ball joints.** They are *modeled* as ball joints but the physical model realize them as a screw through a clearance hole. That is nominally a pin joint; the extra rotational freedom comes from the clearance, so it is a compliant stand-in for a spherical pair rather than a true ball joint.

<p align="center">
  <img src="docs/ball_joint_detail.jpg" width="640">
  <br>
  <em>Physical realizations of the joint connecting link 2 with the plate</em>
</p>

The phisical dimension

```matlab
R_b = 28.65;       % base radius        [mm]
R_p = 84.00;       % plate radius       [mm]
L1  = 80.00;       % link Bi -> Pi     [mm]
L2  = 80.00;       % link Pi -> Mi     [mm]
```

# 3. Electronics

# 3. Ball detection

# 4. PID Control