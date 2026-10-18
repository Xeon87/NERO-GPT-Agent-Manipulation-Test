# Changelog

## V2-6 / GitHub v0.1.0 — Public Release

Public-facing cleanup of the validated V2-5.2 stack.

- preserved frozen V69 motion core;
- preserved fresh-episode reset behavior;
- preserved multi-object completion guard;
- preserved shape-aware manipulation priors;
- preserved one-retry planning adaptation and safety recovery;
- renamed public planner surface from Astra-specific wording to configurable LLM wording;
- added model-neutral environment variables with backward compatibility;
- removed hard-coded dollar pricing table from source; optional pricing file can be supplied externally;
- expanded comments/docstrings around task orchestration, manipulation priors, episode reset, planning recovery, and safety recovery;
- added GitHub README, architecture/state-machine docs, quick start, demo prompts, YouTube checklist, release checker, and third-party notices.

## V2-5.2 — Fresh Episode Reset

- `ZERO ALL` and new task runs no longer inherit stale semantic hover/retry state when no object is held.

## V2-5.1 — Multi-Object / Planning Recovery

- multi-object completion guard;
- wider cube aperture prior;
- recoverable Cartesian pre-grasp planning block;
- post-motion settle/recheck for false-negative hover verification.

## V2-5 — Failure-Aware Recovery

- structured pre-grasp safety evidence;
- explicit retreat / fresh perception / modified retry;
- one retry per object.
