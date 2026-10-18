# NERO V2-6 Public Release Notes

This release is intended for GitHub/YouTube presentation. The underlying manipulation behavior is kept close to the V2-5.2 build that was validated in MuJoCo; the primary changes are public-facing naming, documentation, source comments, configuration hygiene, and reproducibility materials.

## Publication checklist

Before pushing to a public GitHub repository:

- inspect `git diff` and `git status`;
- run `./release_check.sh`;
- confirm no API key appears in history or screenshots;
- decide whether to keep the conservative preview license or replace it after an ownership/license review;
- record one uninterrupted demo run and save the matching runtime log;
- put actual benchmark numbers only in the README after repeated trials, not from a single successful run.
