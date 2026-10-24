---
name: rdd-help
description: >
  Quick-reference for RDD (Requirement Driven Development) workflow and agents.
  Use when asking for RDD help, how to use RDD, or the process.
---

# RDD Quick Ref

## Invoke

Use RDD agents directly:
- `@rdd-planner <description>` — Plan a feature
- `@rdd-reviewer <scope>` — Review code
- `@rdd-agent <problem>` — Debug or adjust behavior

Or select them from the agent picker in the sidebar.

## High Signal Rule (applies to all RDD work)

- Omit any heading, table, or step with no real content.
- No boilerplate sections. No empty summaries.
- Default terse (caveman full) for narrative. Full templates only on explicit "detailed" or "full report".
- Code, R-ids, requirement text, and errors stay normal prose.
- Goal: maximum signal, minimum noise.

## Core Flow (condensed)

1. Frame problem (pain, constraints, success, risks, non-goals).
2. Minimal design (smallest change, files touched).
3. Design gate (show it, wait for approval).
4. Draft requirements (atomic "shall", dupe/anchor checks).
5. Requirements gate (show them, explicit approval).
6. SpecRef write (if MCP ready) or strong local markdown + tagging guidance.
7. Impl plan (R-ids, test targets, timing).
8. Execute (implement, then requirement tests; R-id tags).
9. Review (7 dimensions, only real findings).
10. Rework (root cause or behavior change, handoff to review).

## Key Behaviors

- Auto-activates caveman full for RDD work. "stop caveman" disables.
- Gates are helpful (recommend + full artifact first, override allowed with "proceed anyway").
- Uses native todo_write for live phase tracking.
- SpecRef MCP preferred for traces. Graceful fallback to local docs when absent.
- Trace rules enforced: LLRs need source + unit test; HLRs need integration/e2e test.

## Agents

| Agent | Purpose | Edit Access |
|-------|---------|-------------|
| rdd-planner | Plan features, draft requirements | No |
| rdd-reviewer | Code review across 7 dimensions | No |
| rdd-agent | Debug, root-cause, behavior changes | Yes |
| rdd-bootstrap | Context gathering helper | No |
| rdd-requirements | Requirement analysis helper | No |
| rdd-test-writer | Requirement test helper — test files only | Yes (tests only) |

## Controls

- "stop caveman" / "normal mode"
- "proceed anyway" at a gate
- `@rdd help`

See `.kilo/rules/rdd-core.md` and `.kilo/rules/specref-trace.md` for deeper rules.
