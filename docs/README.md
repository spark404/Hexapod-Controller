# Hexapod-Controller documentation

| Document | What it is | Read it when |
|----------|-----------|--------------|
| [`design.md`](design.md) | **DRAFT for review.** Unified design: requirements, SPI command interface, frames, motion model, gait framework, ground contact, lifecycle | You want to review or challenge the design intent |
| [`control-model.md`](control-model.md) | Earlier design intent: joystick modes (arc / strafe) and the WALKING gait contracts C1–C4. To be superseded by `design.md` once approved | You want to know what the robot is *supposed* to do |
| [`controller-overview.md`](controller-overview.md) | As-built reference: architecture, state machine, math, controller ↔ servo integration | You are changing `controller.c` or the servo task |
| [`controller-audit.md`](controller-audit.md) | Findings: bugs, safety gaps, walking-model gaps, interaction problems, each mapped to a backlog item | You want the reasoning behind a backlog item |
| [`frames-analysis.md`](frames-analysis.md) | How `controller.c` uses coordinate frames today (world, hexapod, body, coxa, servo), 14 findings, and the mapping to the planned world / level / body / leg frames | You are doing the frames part of the refactor (T5.2, T7.8) |
| [`improvement-backlog.md`](improvement-backlog.md) | Six phases of prioritized work items (IDs T1.1–T7.11) with status, dependencies, how to do each, and how to verify | You are deciding what to work on next |
| [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) | Implementation spec for XL430 hardware motion profiles (backlog T4.1 / T4.2) | You are implementing T4.1 / T4.2 (Phase 2) |
| [`servo-profile-bench-experiment.md`](servo-profile-bench-experiment.md) | The bench experiment on servo profiles (backlog T7.14): plan, how it was run, and the results in section 6. The code is on the branch `bench-test` | You want the measured behaviour of the servos, or to decide how T4.1 / T4.2 / T7.15 are done |

Flow: design intent (`control-model`) → as built (`controller-overview`) →
what is wrong (`controller-audit`) → what to do about it (`improvement-backlog`,
with `servo-velocity-control-phase2` as the detailed spec for T4.1 and T4.2).

Superseded documents from May and July 2026 are in [`archive/`](archive/README.md);
their R and D numbers are a different scheme from `design.md`.

Helper scripts live in [`../tools/`](../tools/).
