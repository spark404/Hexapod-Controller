# Archive

Superseded documents, kept for reference. They were never in git before
2026-10-16; they come from an untracked `documentation/` folder that the
consolidation into `docs/` (commit `63aeb36`, 2026-10-10) left behind.

**Their numbering is not the current one.** The R and D numbers below are
separate schemes from those in [`../design.md`](../design.md) (R1–R8, D1–D8),
and the "tracks" are not the phases of [`../improvement-backlog.md`](../improvement-backlog.md).
Line numbers in them refer to the code at the time. Links between them point
to their old names and locations and may be broken.

| Document | Date | Was |
|---|---|---|
| [`controller-redesign-requirements-2026-07.md`](controller-redesign-requirements-2026-07.md) | 2026-07 | Leading document of the July plan: how WALKING and ROTATING actually work, defects D1–D14, hardware checks I1–I4, what to keep, requirements R1–R23 for a redesign |
| [`improvement-backlog-2026-07.md`](improvement-backlog-2026-07.md) | 2026-07 | The July work plan in four tracks, with the disposition of the May backlog |
| [`control-model-2026-07.md`](control-model-2026-07.md) | 2026-07 | The July control model, with a 100 Hz control / 200 Hz servo task split that the current `control-model.md` dropped |
| [`improvement-backlog-2026-05.md`](improvement-backlog-2026-05.md) | 2026-05 | The May backlog |
| [`controller-gaps.md`](controller-gaps.md) | 2026-05 | Severity-tagged robustness audit of the controller and servo task |
| [`controller-servo-interaction-review.md`](controller-servo-interaction-review.md) | 2026-05 | Review of the controller and servo task interaction |
| [`walking-model-review.md`](walking-model-review.md) | 2026-05 | WALKING audit against the earlier fixed 40 mm stride contracts |

The May analyses are the baseline of [`../controller-audit.md`](../controller-audit.md).
