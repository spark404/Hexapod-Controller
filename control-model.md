---

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

This control model is **reasonable, scalable, and well-suited** for a hexapod using tripod gaits and continuous servo control.

If you want, next steps could be:

* Input-to-velocity math
* Mode transition handling
* How this maps onto your existing walking/rotating controller states

## Controller model for servo/control tasks

```text
                 ┌──────────────────────────┐
                 │        Joystick / UI      │
                 │  (x, y, yaw, mode flags)  │
                 └─────────────┬────────────┘
                               │
                               ▼
┌───────────────────────────────────────────────────┐
│                 CONTROL TASK (≈100 Hz)             │
│                                                   │
│  • Reads joystick input                           │
│  • Applies control mode                           │
│    - Arc-based walking                            │
│    - Strafing + independent yaw                   │
│  • Gait & path planning                           │
│  • Inverse kinematics                             │
│  • Uses ACTUAL joint feedback                     │
│                                                   │
│  OUTPUT: Target Joint Angles @ T + 10 ms          │
└─────────────┬─────────────────────────────────────┘
              │   (TargetAngles[T+10ms])
              │
              ▼
┌───────────────────────────────────────────────────┐
│          TARGET BUFFER / SHARED STATE              │
│                                                   │
│  • Double-buffer or queue                         │
│  • Timestamped targets                            │
│  • Overwrite-safe (latest wins)                   │
└─────────────┬─────────────────────────────────────┘
              │
              ▼
┌───────────────────────────────────────────────────┐
│               SERVO TASK (≈200 Hz)                 │
│                                                   │
│  • Reads current servo positions                  │
│  • Reads latest target angles                     │
│  • Interpolates short trajectory (≈5 ms)          │
│  • Enforces velocity / acceleration limits        │
│  • Writes commands to servos                      │
│                                                   │
│  OUTPUT: Incremental servo commands               │
└─────────────┬─────────────────────────────────────┘
              │
              ▼
┌───────────────────────────────────────────────────┐
│           DYNAMIXEL SERVOS (XL430)                 │
│                                                   │
│  • Internal PID                                   │
│  • Quantization & deadband                        │
│  • Return actual position                         │
└─────────────┬─────────────────────────────────────┘
              │
              ▼
┌───────────────────────────────────────────────────┐
│        SERVO FEEDBACK (Actual Angles)              │
│                                                   │
│  • Read via Sync Read                             │
│  • Timestamped                                   │
│  • Used by BOTH loops                             │
└─────────────┴─────────────────────────────────────┘

```