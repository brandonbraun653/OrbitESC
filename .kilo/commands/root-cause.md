---
name: root-cause
description: Root-cause a bug or flaky behavior with the RDD Rework agent.
argument-hint: Describe the bug, failing test, or reproduction path
agent: rdd-agent
---

Debugging mode unless request clearly asks for intentional behavior change.

Prioritize available inputs:

- failing test path or command
- reproduction steps
- suspected files or recent diff
- user-visible impact

Required:

- reproduce with narrowest useful experiment
- confirm or falsify hypotheses — no guessing
- reproduce with a test, command, or script before fix; add a regression test after fix
- smallest robust change only
- flag req impact — no silent semantics changes
