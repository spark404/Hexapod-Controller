# Hexapod Movement Control Model

This document describes the redesigned movement and control concept for the hexapod robot.
The goal is to provide intuitive joystick-based control while keeping gait generation predictable and mechanically safe.

---

## Overview

The hexapod supports two primary locomotion modes:

1. **Arc-Based Locomotion (Default Mode)**
2. **Strafing Locomotion (Alternate Mode)**

A joystick is used as the primary input device.
All motion commands are **proportional**, derived directly from joystick axis values.

---

## 1. Arc-Based Locomotion (Default)

### Concept

* The robot moves using **arc-based motion** in the horizontal plane.
* Movement direction is **relative to the robot’s current facing**.
* Translation and rotation are coupled naturally, similar to how a wheeled robot or animal turns.

### Joystick Mapping

* **X-axis**: Controls turning rate (yaw)
* **Y-axis**: Controls forward/backward velocity
* Joystick magnitude directly scales speed and turn rate (proportional control)

### Behaviour

* Forward joystick → walk forward
* Forward + left/right → follow a curved arc
* Left/right only → rotate in place (small-radius arc)
* Arc radius is **fixed or bounded**, not user-adjustable

### Intended Use

* Primary walking mode
* Intuitive navigation
* Stable gait with predictable foot placement

---

## 2. Strafing Locomotion (Alternate Mode)

### Concept

* Activated by pressing a **mode toggle button**
* Robot movement becomes **independent of current facing**
* The robot can move laterally (sideways) without rotating

### Joystick Mapping (Movement)

* **X-axis**: Left/right strafing
* **Y-axis**: Forward/backward strafing
* Movement is in a **world- or body-aligned frame**, not heading-aligned

### Joystick Mapping (Rotation)

* Separate control (e.g. second joystick axis, trigger, or twist input)
* Controls **yaw rotation independently** of translation

### Behaviour

* Move sideways while facing forward
* Rotate while holding position
* Combine translation and rotation freely

### Intended Use

* Precision positioning
* Alignment tasks
* Confined spaces

---

## Control Scaling and Proportional Input

* The joystick provides **continuous proportional input**
* No discrete speed levels
* Controller derives:

    * Linear velocity directly from joystick magnitude
    * Angular velocity directly from joystick deflection
* Dead zones may be applied near zero input
* Saturation limits are enforced by the controller:

    * Max linear velocity
    * Max yaw rate
    * Max per-foot displacement

---

## Mode Switching

* A button toggles between:

    * Arc-Based Mode
    * Strafing Mode
* Switching modes does **not reset gait state**
* Current leg phase continues smoothly

---

## Controller Responsibilities

The controller must:

* Convert joystick input into:

    * Body-frame or world-frame velocity vectors
    * Desired yaw rate
* Enforce velocity and acceleration limits
* Ensure:

    * Grounded legs do not slip
    * Swing legs complete their paths before phase swap
* Maintain tripod gait consistency

---

## Why This Model Works Well

* Matches human intuition (walk vs strafe)
* Scales naturally with proportional controls
* Keeps gait generation simple and reusable
* Separates **what the user wants** from **how the robot executes it**

---

## Summary

| Feature                | Arc Mode | Strafe Mode |
| ---------------------- | -------- | ----------- |
| Facing-relative motion | Yes      | No          |
| Sideways movement      | No       | Yes         |
| Independent rotation   | Limited  | Yes         |
| Default navigation     | Yes      | No          |
| Precision positioning  | No       | Yes         |


---

## Walking gait contracts

The WALKING state implements the arc-based mode with a **fixed 40 mm stride**
in the X/Y plane around each leg's neutral position (`tip_home`), regardless of
body height. The robot moves at the commanded speed by running a tripod-gait
lift/stand cycle; a lower commanded speed simply means a longer cycle time —
the swing always covers the full stride, it just takes longer. New movement
instructions are incorporated immediately, without waiting for the current
cycle to complete, taking the current leg positions into account to decide what
motion is feasible.

That decomposes into four contracts the controller must keep:

| #  | Contract |
|----|----------|
| C1 | Each cycle, every leg performs a full 40 mm stride around `tip_home` in body-frame XY. |
| C2 | The body moves at the commanded linear (and angular) speed *averaged over a cycle* — at lower speeds the cycle stretches in time, not in stride. |
| C3 | Movement is a tripod-gait swing/stance cycle. |
| C4 | New `(velocity, heading)` commands are absorbed without finishing the current cycle, and the controller respects the current leg geometry when doing so. |

The constant-stride choice is deliberate: it keeps gait geometry invariant
across speeds (every cycle is the same, only timing changes), gives a
predictable landing impact profile, and avoids the "shuffle" failure mode of
proportional-stride walkers at low speed.

How well the implementation keeps these contracts, and where the model itself
has gaps, is evaluated in [`controller-audit.md`](controller-audit.md) §3.

---

## Task structure

The control task plans (gait, path, inverse kinematics) from joystick commands
and actual joint feedback, and publishes target joint angles; the servo task
turns those targets into servo commands and publishes the actual positions
back. The as-built loop rates, shared state and timing are documented in
[`controller-overview.md`](controller-overview.md) §1 and §6. (The original
design sketch of a 100 Hz / 200 Hz split with software interpolation in the
servo task has been superseded; see backlog T4.1 and T4.2 for the move to XL430
hardware motion profiles.)

---

## Implementation status

Arc-based mode is implemented. Strafing mode, the mode toggle, and an
independent yaw-rate input are not (backlog T6.1).
