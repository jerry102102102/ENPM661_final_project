# Documentation Index

The project documents are split into two groups.

## Paper Content

Use this folder when writing the final report, paper sections, slides, or oral
presentation content.

Folder:

```text
doc/paper_content/
```

It contains:

- method explanation
- ACTEA definition
- experiment result interpretation
- polished English report draft
- related-work notes
- prior-work relationship and extension framing

Recommended reading order:

1. `paper_content/actea_evaluation_report_polished.md`
2. `paper_content/relation_to_prior_work_and_our_extensions.md`
3. `paper_content/phase3_cached_temporal_roadmap.md`
4. `paper_content/actea_evaluation_report.md`
5. `paper_content/t_prm_paper_notes_and_implementation_plan.md`

## Implementation Process

Use this folder when checking how the project was built, what was planned, what
changed in each phase, and what implementation tasks were completed.

Folder:

```text
doc/implementation_process/
```

It contains:

- project plans
- implementation specs
- phase writeups
- architecture/refactor notes
- roadmap builder status

Recommended reading order:

1. `implementation_process/nonholonomic_temporal_roadmap_project_plan.md`
2. `implementation_process/nonholonomic_ta_prm_spec.md`
3. `implementation_process/refactor_phase1_architecture.md`
4. `implementation_process/sampled_nonholonomic_phase1_status.md`
5. `implementation_process/phase2_temporal_roadmap_pipeline.md`
6. `implementation_process/core_method_completion_and_evaluation_spec.md`

## Quick Rule

If the file helps answer:

```text
What should we write in the report?
```

put it under `paper_content/`.

If the file helps answer:

```text
How did we build or plan the code?
```

put it under `implementation_process/`.

## Local Notes Not Pushed to Main

Chinese drafts and local explanation notes are kept under:

```text
doc/local_zh_notes/
```

That folder is ignored by git and should not be pushed to `main`.

## Demo Video

To render a short 2D demo video:

```bash
python3 scripts/render_actea_demo_video.py
```

Default outputs:

```text
outputs/demo/actea_2d_demo.gif
outputs/demo/actea_2d_demo.mp4
```

The demo shows a single ACTEA-planned query with moving circular obstacles and
the robot reaching the goal in a 5-second animation.
