# Duet3Expansion design notes

Background and reasoning for parts of this firmware where the code alone does not explain why it is
shaped the way it is. These exist so the source can stay focused on *what the code does* and *what must
not be changed*, without also carrying the story of how each conclusion was reached.

## What lives where

**In the source:** anything that stops a future edit from reintroducing a bug — invariants, ordering
constraints, "deliberately not X because Y". If deleting a comment would let someone silently break the
code, that comment belongs next to the code.

**In these documents:** how we found out. Symptoms, measurements, the hypotheses that turned out wrong,
and the numbers that settled it. Useful when something changes, unhelpful when you are reading the
function to see what it does.

## Cross-referencing convention

- Source files point here as `// Background: docs/<file>.md#<anchor>`.
- Documents point at **files and symbols**, never line numbers — `FocCurrentSense::Poll()`, not
  `FocCurrentSense.cpp:236`. Line numbers rot on the first edit; symbol names survive refactors and are
  greppable.

## Index

| document | covers |
|---|---|
| [foc-commutation.md](foc-commutation.md) | alignment reference, encoder direction, counts per electrical revolution and why it must be exact, two limits that were removed |
| [foc-current-sense.md](foc-current-sense.md) | ADC ownership, PWM-synchronous sampling, the DSEQ scan, frame calibration, saturation and staleness |
| [foc-current-control.md](foc-current-control.md) | d/q loops, limiting and anti-windup, motor parameters, tuning, what `O` now does |
| [closed-loop-cascade.md](closed-loop-cascade.md) | the three-branch restructure, stall detection, SVPWM |
| [board-config.md](board-config.md) | SAMME51 pin and peripheral allocation, ADC reference, DMA channels |
| [telemetry.md](telemetry.md) | M569.5 channels: the ordering invariant, sign convention, and channel quirks |

Related, in the sibling repository: `CANlib/doc/generic-message-tables.md` covers the constraints on
extending a `M569.x` parameter table — the 20-bit `paramMap`, the unusable `G`/`M` letters, and retired
letters.

## Session change descriptions

The `CHANGES_*.md` files in the RRFBuild root describe what changed in a particular working session.
These documents describe how the code *is*, and are the ones to keep current.
