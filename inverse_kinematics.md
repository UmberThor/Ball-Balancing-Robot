# Inverse Kinematics

Documentation of the inverse-kinematics solution implemented in `ball_joints.m`
(with hand-written derivation in `inverse kinematics notes.pdf`).

## Goal

Given a desired **pose of the platform**, compute the **three motor angles** at the
base that realize it.

- **Input (pose):** unit normal vector of the platform `n = [α, β, γ]` and height
  `h` of the platform center.
- **Output:** motor angles `q1, q2, q3`.

The platform center is constrained to the vertical `z`-axis at `(0, 0, h)`; the
platform can only **tilt** (via `n`) and **translate vertically** (via `h`). This
gives 3 DOF, matched by the 3 motors.

## Mechanism

Three identical arms, spaced 120° apart. Each arm `i` is a 2-link chain:

```
platform  Bi --- L1 --- Pi --- L2 --- Mi  base
          (ball joint) (elbow)     (motor)
```

- `Mi` — motor point, fixed on the base circle (radius `R_b`).
- `Pi` — elbow joint between the two links.
- `Bi` — ball joint on the platform circle (radius `R_p`).
- `L1` — connecting rod (platform side, `Bi–Pi`).
- `L2` — motor arm (base side, `Pi–Mi`), driven by the motor.

Each arm lives in a fixed **vertical plane** passing through the `z`-axis and its
motor `Mi`. This is the key that makes the problem tractable: every point of arm `i`
(`Mi`, `Pi`, `Bi`) satisfies `y = Eᵢ·x`.

### Parameters (current test values)

| Symbol | Meaning              | Value |
|--------|----------------------|-------|
| `R_b`  | base radius          | 1     |
| `R_p`  | platform radius      | 1.25  |
| `L1`   | connecting rod       | 1     |
| `L2`   | motor arm            | 1.25  |
| `h`    | platform height      | 2     |
| `n`    | platform normal      | [0,0,1] |

### Base points and arm planes

The motors sit on the base circle at angles 180°, 300°, 60°:

| Arm | `Mi = R_b·(cosθ, sinθ, 0)` | Arm plane | `Eᵢ` |
|-----|----------------------------|-----------|------|
| 1   | θ = π    → `(-R_b, 0, 0)`             | `y = 0`      | `0`   |
| 2   | θ = 5π/3 → `(R_b/2, -R_b√3/2, 0)`     | `y = -√3 x`  | `-√3` |
| 3   | θ = π/3  → `(R_b/2,  R_b√3/2, 0)`     | `y = √3 x`   | `√3`  |

## Step 1 — Ball joint positions `Bi(n, h)`

Each `Bi` is the intersection of three conditions:

1. **On the platform plane.** A point `P` lies on the platform iff `(P − c)·n = 0`
   with `c = (0,0,h)`:
   ```
   α x + β y + γ z − h γ = 0
   ```
2. **In its arm plane** `y = Eᵢ x`.
3. **On the platform rim:** distance `R_p` from the center,
   `x² + y² + (z − h)² = R_p²`.

Solving the system per arm gives closed forms (implemented directly in the script):

**B1** (`y = 0`):
```
x = −γ R_p / √(α² + γ²)
y = 0
z = h + α R_p / √(α² + γ²)
```

**B2** (`y = −√3 x`):
```
x =  γ R_p / √(4γ² + (√3β − α)²)
y = −√3 γ R_p / √(4γ² + (√3β − α)²)
z = h + R_p (√3β − α) / √(4γ² + (√3β − α)²)
```

**B3** (`y = √3 x`):
```
x =  γ R_p / √(4γ² + (√3β + α)²)
y =  √3 γ R_p / √(4γ² + (√3β + α)²)
z = h − R_p (√3β + α) / √(4γ² + (√3β + α)²)
```

**Reachability** is guarded by `assert(|L1−L2| < |Bi−Mi| < L1+L2)` per arm.

## Step 2 — Elbow positions `Pi`

`Pi` is fixed by three conditions:

1. **Distance `L1` from `Bi`:** `(x−Bx)² + (y−By)² + (z−Bz)² = L1²`
2. **Distance `L2` from `Mi`:** `(x−Mx)² + (y−My)² + (z−Mz)² = L2²`
3. **In the arm plane** `y = Eᵢ x`.

**Subtracting the two spheres** removes the quadratic terms and yields a plane
`z = A + Bx + Cy`, with:
```
B = (Bx − Mx) / (Mz − Bz)
C = (By − My) / (Mz − Bz)
A = (Mx² − Bx² + My² − By² + Mz² − Bz² + L1² − L2²) / (2 (Mz − Bz))
```

Intersecting this plane with `y = Ex` collapses the elbow to a **line**:
```
y = E x
z = A + (B + C E) x
```

Substituting the line into the "distance `L1` from `Bi`" sphere gives a **quadratic
in `x`**:
```
[1 + E² + (B+CE)²] x²
  + [−2Bx − 2E·By + 2(B+CE)(A−Bz)] x
  + [Bx² + By² + (A−Bz)² − L1²] = 0
```

Solved with the quadratic formula, then `Pi = [x, Ex, A + (B+CE)x]`.

**Root selection.** The quadratic has two roots (elbow up / elbow down). The script
hard-codes the physical branch per arm: **minus** sign for arm 1, **plus** sign for
arms 2 and 3.

## Step 3 — Motor angles `qi`

The motor angle is the tilt of the motor arm `Mi → Pi` measured from the **vertical**:
horizontal projection over vertical rise.
```
qi = atan( √((Pix − Mix)² + (Piy − Miy)²) / |Piz − Miz| )
```

## Visualization

`ball_joints.m` plots the whole assembly: base circle + motor points, platform
circle + normal arrow, ball joints, elbows, and both links of each arm. The helper
`plotCircle3D(center, normal, radius, style)` draws a circle in 3D from a center,
a normal, and a radius (using `null(normal)` for an in-plane basis).