# Paper Content

These files are for writing the final paper/report, presentation, and method
explanation.

## Recommended Reading Order

Read these files in this order if you are preparing the final paper:

1. `actea_evaluation_report_polished.md`
   Clean English report draft. Start here for the full story and usable
   wording.

2. `relation_to_prior_work_and_our_extensions.md`
   Prior-work relationship and contribution framing. Use this for the related
   work and "what we changed" sections.

3. `phase3_cached_temporal_roadmap.md`
   ACTEA method details, edge-level temporal metadata, annotation modes, and
   planner integration.

4. `method_math_and_algorithm_notes.md`
   Paper-facing math and algorithm notes for the nonholonomic roadmap,
   temporal search, ACTEA interval derivation, and heuristic.

5. `actea_evaluation_report.md`
   Longer working report with more detailed interpretation, caveats, and
   experiment discussion.

6. `t_prm_paper_notes_and_implementation_plan.md`
   Background notes on T-PRM / TA-PRM. Use selectively for related work.

## Main Report Drafts

- `actea_evaluation_report_polished.md`
  English polished version. Best source for final report wording.

- `actea_evaluation_report.md`
  Longer working report with detailed interpretation and caveats.

## Method and Math

- `phase3_cached_temporal_roadmap.md`
  ACTEA method explanation, edge-level temporal metadata, annotation modes, and
  planner integration.

- `method_math_and_algorithm_notes.md`
  Nonholonomic state/action model, static roadmap construction, temporal
  labels, ACTEA interval derivation, edge-time cost, and heuristic notes.

## Background / Related Work

- `relation_to_prior_work_and_our_extensions.md`
  English version for the related-work and contribution-framing sections.

- `t_prm_paper_notes_and_implementation_plan.md`
  Notes on T-PRM / TA-PRM background and how the project adapts the temporal
  roadmap idea.

## What to Use for Final Writing

Use these sections directly:

- problem motivation
- ACTEA definition
- temporal edge-validity explanation
- correctness experiment
- repeated-query experiment
- hard dynamic-scene comparison
- roadmap-scale ablation
- limitations and caveats

## Local Chinese Notes

Chinese drafts are kept locally under:

```text
doc/local_zh_notes/
```

That folder is ignored by git and should not be pushed to `main`.
