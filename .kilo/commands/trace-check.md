---
name: trace-check
description: Run a focused SpecRef tagging and traceability audit on files, selections, or the current diff.
argument-hint: Provide paths, requirement IDs, or say current diff
agent: RDD Reviewer
---

Focus on:

- SpecRef tag placement
- source-to-test R-id coverage
- trace integrity and orphan detection
- missing requirements for non-trivial logic

Use current diff when no explicit files provided.

Keep non-trace dims brief unless they reveal a blocker.
