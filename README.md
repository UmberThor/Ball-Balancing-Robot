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

Reference implementation: `ball_joints.m`.

## 1.1 Problem statement

The plate is a rigid disc whose center is constrained to the vertical $z$ axis. It can therefore only *tilt* and *rise*, which is exactly 3 DOF.

**Input**: the pose of the plate:

- $\mathbf{n} = [\alpha,\ \beta,\ \gamma]^T$, the unit normal of the plate,
- $h$, the height of the center of the plane $c = (0, 0, h)$.

**Output**: the three motor angles $q_1, q_2, q_3$.


## 1.2 Architecture

The actuation mechanism consists of three identical arms, located 120° apart. Each arm $i$ is a two-link chain that connects the base to the plate. Each arm is a **3-RRS** architecture: revolute at the shoulder, revolute at the elbow, and spherical at the wrist. The shoulder has the motor, the elbow has a pin joint and the wrist has a ball joint.

<p align="center">
  <img src="docs/Figure_1.png" width="640">
  <br>
  <em>Architecture of the 3-DOF manipulator for <b>n</b> = [&minus;0.25, 0.33, 1]<sup>T</sup>, plotted by <code>ball_joints.m</code></em>
</p>


| Point / length | Meaning |
|---|---|
| $M_i$ | coordinates of the $i$-th motor, fixed on the base circle of radius $R_b$ |
| $P_i$ | coordinates of the $i$-th pin joint connecting the two links |
| $B_i$ | coordinates of the $i$-th ball joint on the plate circle of radius $R_p$ |
| $L_1$ | length of the lower link (link 1) that connects $M_i \to P_i$ |
| $L_2$ | length of the upper link (link 2) that connects $P_i \to B_i$ |

> **On the ball joints.** They are *modeled* as ball joints but the physical model realize them as a screw through a clearance hole. That is nominally a pin joint; the extra rotational freedom comes from the clearance, so it is a compliant stand-in for a spherical pair rather than a true ball joint.

<p align="center">
  <img src="docs/placeholder.gif" width="640">
  <br>
  <em>Physical realizations of the joint connecting link 2 with the plate</em>
</p>

The phisical dimension 

```matlab
R_b = 28.65;       % base radius        [mm]   
R_p = 84.00;       % plate radius       [mm]
L1  = 80.00;       % rod   Bi -> Pi     [mm]
L2  = 80.00;       % crank Pi -> Mi     [mm]
```

## 1.3 The one idea that makes this solvable

Each motor rotates about a **horizontal** axis, so its whole arm ($M_i$, $P_i$ and
the plate joint $B_i$ it drives) is confined to a **fixed vertical plane containing
the $z$ axis**. In coordinates, that plane is simply

$$y = E_i\, x .$$

So the entire 3D problem collapses into three independent 2D problems, one per
plane. Everything that is arm-specific is captured by two numbers:

$$E_i = \tan\theta_i \quad\text{(which plane)}, \qquad
s_i = \operatorname{sign}(\cos\theta_i) \quad\text{(which half of it)},$$

where $\theta_i$ is the angular position of motor $i$ on the base circle,
$M_i = R_b\,(\cos\theta_i,\ \sin\theta_i,\ 0)$. The arm lives on the half-plane on
the same side of the $z$ axis as its own motor, and that side is the sign $s_i$.

| Arm | $\theta_i$ | $M_i$ | plane | $E_i$ | $s_i$ |
|-----|-----------|-------|-------|-------|-------|
| 1 | $\pi$ | $(-R_b,\ 0,\ 0)$ | $y = 0$ | $0$ | $-1$ |
| 2 | $5\pi/3$ | $(R_b/2,\ -\tfrac{\sqrt3}{2}R_b,\ 0)$ | $y = -\sqrt3\,x$ | $-\sqrt3$ | $+1$ |
| 3 | $\pi/3$ | $(R_b/2,\ +\tfrac{\sqrt3}{2}R_b,\ 0)$ | $y = +\sqrt3\,x$ | $+\sqrt3$ | $+1$ |

```matlab
M1 = [R_b * cos(pi);     R_b * sin(pi);     0];
M2 = [R_b * cos(5*pi/3); R_b * sin(5*pi/3); 0];
M3 = [R_b * cos(pi/3);   R_b * sin(pi/3);   0];
```

The script writes each arm out explicitly with its own $E_i$ and its own $\pm$
sign; those hard-coded numbers are nothing more than the $(E_i, s_i)$ pair of the
table above.

## 1.4 Step 1 — plate joints $B_i(\mathbf{n}, h)$

$B_i$ is the intersection of three conditions:

1. **it lies on the plate plane** — $(\mathbf{p} - c) \cdot \mathbf{n} = 0$, i.e.

$$\alpha x + \beta y + \gamma z - h\gamma = 0$$

2. **it lies in the arm plane** — $y = E_i x$

3. **it lies on the plate rim** — $x^2 + y^2 + (z - h)^2 = R_p^2$

Substituting (2) into (1) gives the height along the plane as a function of $x$:

$$z = h - \frac{\alpha + \beta E_i}{\gamma}\,x$$

and substituting both into (3) leaves a pure quadratic in $x$:

$$x^2\left[\,1 + E_i^2 + \frac{(\alpha + \beta E_i)^2}{\gamma^2}\right] = R_p^2 .$$

Hence the **closed form for any arm**:

$$\boxed{\;
x_i = s_i\,\frac{\gamma R_p}{\sqrt{\gamma^2\left(1 + E_i^2\right) + (\alpha + \beta E_i)^2}},
\qquad y_i = E_i\,x_i,
\qquad z_i = h - \frac{\alpha + \beta E_i}{\gamma}\,x_i \;}$$

The sign $s_i$ picks the one of the two rim points that is on the motor's side —
the other root is the diametrically opposite point of the rim.

> **What this costs.** Placing each $B_i$ where its arm plane cuts the rim keeps
> the three points on the rim circle, but not exactly 120° apart on it: the
> triangle $B_1B_2B_3$ stretches slightly as the plate tilts (sides 145.5 mm when
> level, 143.5–147.3 mm at a 20° tilt). A perfectly rigid plate with perfect
> joints could not do that. It is the same clearance and compliance discussed in
> §1.2 that absorbs the difference, and below ~6° of tilt — the range the
> controller actually commands — the mismatch is under 0.2 %.

Expanding the box for the three arms gives exactly what is coded in
`ball_joints.m`:

**Arm 1** — $E_1 = 0$, $s_1 = -1$:

$$B_1 = \left(-\frac{\gamma R_p}{\sqrt{\alpha^2 + \gamma^2}},\;\; 0,\;\;
h + \frac{\alpha R_p}{\sqrt{\alpha^2 + \gamma^2}}\right)$$

**Arm 2** — $E_2 = -\sqrt3$, $s_2 = +1$ (so $1 + E^2 = 4$ and
$\alpha + \beta E = \alpha - \sqrt3\beta$):

$$B_2 = \left(\frac{\gamma R_p}{d_2},\;\;
-\frac{\sqrt3\,\gamma R_p}{d_2},\;\;
h + \frac{R_p\left(\sqrt3\beta - \alpha\right)}{d_2}\right),
\qquad d_2 = \sqrt{4\gamma^2 + \left(\sqrt3\beta - \alpha\right)^2}$$

**Arm 3** — $E_3 = +\sqrt3$, $s_3 = +1$:

$$B_3 = \left(\frac{\gamma R_p}{d_3},\;\;
+\frac{\sqrt3\,\gamma R_p}{d_3},\;\;
h - \frac{R_p\left(\sqrt3\beta + \alpha\right)}{d_3}\right),
\qquad d_3 = \sqrt{4\gamma^2 + \left(\sqrt3\beta + \alpha\right)^2}$$

> **Singularity.** Every formula divides by $\gamma$ (or by a $d_i$ that vanishes
> with it): the plate may never be exactly vertical, $\gamma \neq 0$. Not a
> practical restriction — the useful range is $\gamma \approx 1$.

## 1.5 Reachability check

Before solving for the elbow, the two-link chain must be able to span the gap
between motor and plate joint. The triangle inequality gives, per arm,

$$|L_1 - L_2| < \lVert B_i - M_i \rVert < L_1 + L_2$$

```matlab
assert(norm(B1-M1)<L1+L2 & norm(B1-M1)>abs(L1-L2));
```

If it fails, the requested pose $(\mathbf{n}, h)$ is simply not reachable. In the
MATLAB script this aborts; in `ball_balancer.py` the same test is a soft guard —
the frame is skipped and the servos hold their last command.

## 1.6 Step 2 — elbows $P_i$

$P_i$ is again the intersection of three conditions:

1. **at distance $L_1$ from the plate joint** — $(x - B_x)^2 + (y - B_y)^2 + (z - B_z)^2 = L_1^2$
2. **at distance $L_2$ from the motor** — $(x - M_x)^2 + (y - M_y)^2 + (z - M_z)^2 = L_2^2$
3. **in the arm plane** — $y = E_i x$

### Two spheres → a plane

Subtracting sphere (2) from sphere (1) kills every quadratic term and leaves a
plane, which solved for $z$ reads

$$z = A + Bx + Cy$$

$$A = \frac{L_1^2 - L_2^2 + \left(M_x^2 - B_x^2\right) + \left(M_y^2 - B_y^2\right) + \left(M_z^2 - B_z^2\right)}{2\,(M_z - B_z)},
\qquad B = \frac{B_x - M_x}{M_z - B_z},
\qquad C = \frac{B_y - M_y}{M_z - B_z}$$

(Geometrically this is the *radical plane* of the two spheres: the plane that
contains their intersection circle.)

### Plane ∩ arm plane → a line

Intersecting it with $y = E x$ collapses the circle to the two points of a line:

$$y = E x, \qquad z = A + (B + CE)\,x$$

### Line ∩ sphere → a quadratic in $x$

Putting the line back into condition (1) gives

$$\underbrace{\left[1 + E^2 + (B + CE)^2\right]}_{a}x^2
+ \underbrace{\left[-2B_x - 2EB_y + 2(B + CE)(A - B_z)\right]}_{b}x
+ \underbrace{\left[B_x^2 + B_y^2 + (A - B_z)^2 - L_1^2\right]}_{c} = 0$$

$$x_{1,2} = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a},
\qquad P_i = \Big(x,\;\; E x,\;\; A + (B + CE)\,x\Big)$$

### Which root?

The two roots are the two physical assemblies of the arm: **elbow pointing outward**
(away from the $z$ axis) and **elbow folded inward**. The robot is built with the
elbow out, so the root to keep is the one **farther from the axis on the motor's
side** — that is the $+$ root when $s_i = +1$ and the $-$ root when $s_i = -1$:

```matlab
P1x = (-b - sqrt(b^2 - 4ac)) / (2a);   % arm 1: s = -1, motor at x < 0
P2x = (-b + sqrt(b^2 - 4ac)) / (2a);   % arm 2: s = +1, motor at x > 0
P3x = (-b + sqrt(b^2 - 4ac)) / (2a);   % arm 3: s = +1, motor at x > 0
```

So the same per-arm sign $s_i$ of §1.3 selects both the plate joint in Step 1 and
the elbow branch in Step 2. (`ball_balancer.py` factors this out into the single
function `solve_P(M, B, E, sign)`, called with `(M1, B1, 0, -1)`,
`(M2, B2, -√3, +1)`, `(M3, B3, +√3, +1)`.)

## 1.7 Step 3 — motor angles $q_i$

The motor angle is the tilt of the crank $M_i \to P_i$ measured **from the
vertical**: horizontal run over vertical rise.

$$q_i = \arctan\!\left(\frac{\sqrt{(P_{ix} - M_{ix})^2 + (P_{iy} - M_{iy})^2}}{\lvert P_{iz} - M_{iz}\rvert}\right)$$

```matlab
q1 = atan(sqrt((P1(2)-M1(2))^2 + (P1(1)-M1(1))^2)/abs(P1(3)-M1(3)))
```

$q_i \in [0°, 90°)$: $0°$ is the crank pointing straight up, and it grows as the
crank lies down. The absolute value discards the sign of the vertical component,
so this formula assumes the elbow stays **above** the base plane — true throughout
the working range, and the reachability check of §1.5 plus the servo limits keep
it there.

## 1.8 Worked example

Level plate, $\mathbf{n} = [0, 0, 1]$, $h = 100$, with the constants of §1.2:

| | $B_i$ | $\lVert B_i - M_i\rVert$ | $P_i$ | $q_i$ |
|---|---|---|---|---|
| 1 | $(-84.00,\ 0,\ 100)$ | $103.28$ | $(-130.26,\ 0,\ 34.73)$ | $64.27°$ |
| 2 | $(42.00,\ -72.75,\ 100)$ | $103.28$ | $(65.13,\ -112.81,\ 34.73)$ | $64.27°$ |
| 3 | $(42.00,\ +72.75,\ 100)$ | $103.28$ | $(65.13,\ +112.81,\ 34.73)$ | $64.27°$ |

Three equal angles, as symmetry demands — a good sanity check after any change to
the constants. Tilting the plate with the alternative normal commented in the
script, $\mathbf{n} \propto [0.25,\ 0.25,\ 1]$, breaks the symmetry as expected:
$q_1 = 50.93°$, $q_2 = 59.56°$, $q_3 = 77.98°$.

## 1.9 Visualization

Running `ball_joints.m` draws the whole assembly: base circle and the three motor
points, plate circle with its normal arrow, the three plate joints, the elbows, and
both links of every arm — color-coded red / green / blue for arms 1 / 2 / 3. The
commented-out `surf` blocks draw the three arm planes and the plate plane, useful
when checking that every arm really stays in its own plane.

The plate circle is drawn by the helper

```matlab
plotCircle3D(center, normal, radius, style)
```

which builds an orthonormal basis of the plane with `null(normal)` and sweeps
$\theta \in [0, 2\pi]$ over it.

## 1.10 From MATLAB to the robot

`ball_balancer.py` runs the identical equations on the Raspberry Pi, with the PID
output supplying the tilt: $\mathbf{n} = [u_x,\ u_y,\ 1]$ normalized, at a fixed
$h = 100$. Two things to keep in mind when comparing the two files:

- **The link lengths do not match yet.** MATLAB uses $L_1 = L_2 = 80$, the Python
  file still has $L_1 = L_2 = 50$, flagged `TO BE UPDATED FROM THE 3D MODEL`. The
  angles are meaningless until both are measured off the CAD.
- **The servos work in degrees with an offset.** $q_i$ is converted to degrees,
  clamped to `Q_MIN..Q_MAX` = 35°..75°, and shifted by a per-servo `OFFSETS` to
  absorb the horn mounting. The level pose computed above, $64.27°$, is the
  natural home position and sits inside that range.

---

*Next sections to document: ball detection, PID control loop, mechanical design and
electronics.*
