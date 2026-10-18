# Benchmarking

A single successful demo is useful evidence that the integrated stack can work; it is not enough to characterize reliability.

For public results, run repeated trials from a defined initial scene and record at least:

- task prompt;
- model ID / reasoning setting;
- number of requested objects;
- task success / failure;
- first-try grasp success;
- planning-recovery count;
- safety-recovery count;
- placement-verification success;
- total task time;
- failure category.

Use `benchmarks/results_template.csv` as a starting point. Keep raw logs for failed trials so the published success rate is auditable.

Suggested early comparisons:

1. fixed narrow aperture vs manipulation prior;
2. manipulation prior vs prior + planning recovery;
3. prior + recovery across cube and sphere objects;
4. multi-object completion with different LLM models / reasoning levels.
