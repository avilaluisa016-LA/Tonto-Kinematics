# Project Tonto — 4-DOF Robot Arm Kinematics

Forward kinematics derivation, verification, and interactive visualization for a 4-DOF robotic arm, using Denavit-Hartenberg (DH) parameters.

## Overview

Tonto is a 4-degree-of-freedom robotic arm consisting of:
- A **base joint** that yaws the entire arm side-to-side (turntable-style rotation about a vertical axis)
- Three **sweep joints** (shoulder, elbow, wrist) that pivot the arm up and down within a vertical plane

Because the base joint's rotation axis is perpendicular to the three sweep joints' axes, the DH chain includes a 90° twist at the base — a detail that initially caused a derivation error (see **Verification** below) before being caught via symbolic cross-checking.

## Robot Configuration

| Link | Length (in) | Description |
|------|------|-------------|
| h  | 1.625 | Base-to-shoulder offset |
| L1 | 5.75  | Shoulder-to-elbow |
| L2 | 5.25  | Elbow-to-wrist |
| L3 | 1.5   | Wrist-to-end-effector |

## DH Parameter Table

| Frame | a (length) | α (twist) | d (offset) | θ (angle) |
|-------|-----------|-----------|------------|-----------|
| 1 | 0  | π/2 | 0 | θ1 (base yaw) |
| 2 | h  | 0   | 0 | θ2 (shoulder) |
| 3 | L1 | 0   | 0 | θ3 (elbow) |
| 4 | L2 | 0   | 0 | θ4 (wrist) |
| 5 | L3 | 0   | 0 | 0 (fixed end-effector offset) |

**Note on joint angles:** each θ is a *relative* rotation from the previous link's orientation, not an absolute angle from vertical/horizontal. For example, setting θ2=θ3=θ4=90° does **not** point all three segments straight up — it compounds three successive 90° bends (up, then back, then down). To fully extend the arm vertically, use θ2=90°, θ3=0, θ4=0.

## Forward Kinematics

The end-effector position reduces to a clean closed form. Letting the cumulative angle up to joint *i* be φ2 = θ2, φ3 = θ2+θ3, φ4 = θ2+θ3+θ4:

```
r = h·cos(φ2) + L1·cos(φ3) + (L2+L3)·cos(φ4)
x = r·cos(θ1)
y = r·sin(θ1)
z = h·sin(φ2) + L1·sin(φ3) + (L2+L3)·sin(φ4)
```

Notably, **z is independent of θ1** — the base joint only yaws the arm and cannot change its height, which is a useful physical sanity check on the derivation.

Maximum reach (fully extended, θ2=θ3=θ4=0): **h + L1 + L2 + L3 = 14.125 in**

## Files

- `tonto_kinematics.m` — MATLAB script with two parts:
  1. **Symbolic derivation**: builds T01–T45 from the DH table, computes and simplifies T05, and prints the closed-form x/y/z equations.
  2. **Numeric simulation**: prompts for joint angles (radians), computes the resulting end-effector position, and renders the arm's pose in a live 3D plot.
- `tonto_kinematics.c` *(planned)* — C port of the forward kinematics, intended as groundwork for a reusable embedded FK/IK library.

## Usage (MATLAB)

Run `tonto_kinematics.m`. It will first print the symbolic DH matrices and FK equations, then prompt interactively:

```
Enter theta1 (rad): 0
Enter theta2 (rad): pi/2
Enter theta3 (rad): 0
Enter theta4 (rad): 0
```

A 3D plot of the arm's pose will render, followed by the computed end-effector coordinates. You'll be prompted to try additional poses.

## Verification

The hand-derived DH transformation matrices were cross-checked against a symbolic MATLAB derivation. This caught a sign error in T01's (2,3) entry (a dropped cos θ1 factor, originally written as a constant −1), which had propagated into every downstream transform. After correcting it, the result was confirmed both symbolically and against the physical expectation that end-effector height (z) is independent of θ1, since the base joint only produces yaw.

Numeric test cases (including angles beyond [0, 2π) and negative values) were also checked by hand against the closed-form x/y/z equations above, with agreement to within floating-point precision.

## Future Work

- Port forward kinematics to C (`tonto_kinematics.c`) for embedded deployment on the arm's microcontroller (Feather M0 / PCA9685 servo driver)
- Derive and implement inverse kinematics
- Fold both FK and IK into a reusable, general-purpose C kinematics library for future robotic arm projects
