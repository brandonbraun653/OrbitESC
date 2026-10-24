---
name: rdd
description: >
  Run Requirement Driven Development workflows. Use for features, refactors, debugging, or any work that benefits from explicit requirements, minimal design, traceability, requirement-validation tests, and review.
  Supports modes: plan, review, implement, help.
  Auto-activates high-signal terse output and caveman mode.
---

# RDD Skill — High Signal Contract

## Mandatory Rules (all modes, all subagents)

1. **High Signal First**
   - Never emit a heading, table row, or numbered step that has no substantive content.
   - Omit entire sections when empty. No "N/A" placeholders or boilerplate.
   - Default output is condensed + caveman terse for narrative.
   - Only use full structured templates (exact tables, every phase listed, 7-dimension summary with zeros) when user explicitly says "full report", "detailed", "audit trail", or equivalent.
   - Goal: user sees only decisions, findings, R-ids, and next actions.

2. **Caveman Activation**
   - At the start of any RDD work, load and apply the local `caveman` skill at **full** intensity.
   - Narrative, findings, plans, and recommendations use caveman style.
   - Requirement text, code snippets, R-ids (`R42`), error messages, and file paths stay in normal prose.
   - User can say "stop caveman" or "normal mode" to override.

3. **Todo Discipline**
   - Use `todo_write` immediately to scaffold the canonical phases for the active mode.
   - Keep exactly one item `in_progress` at a time.
   - Mark complete as soon as the phase finishes.

4. **Gates**
   - Design and requirements buyoff gates use `question`.
   - Always show the full artifact in the chat first, _then_ ask the short approval question.
   - Gates are "helpful" (strong recommendation). User may override with "proceed anyway". Note the override.

5. **SpecRef & Traceability**
   - Prefer SpecRef MCP tools when the server is configured (use `search_tool` then `use_tool`).
   - When unavailable: produce excellent local requirement documents + clear `// R<id>` or `@req R<id>` tagging guidance.
   - Enforce the rules from the local contracts (see below).

6. **Requirement Tests & Minimal Change**
   - Tests must validate requirement outcomes. Default timing is after implementation; use rdd-test-writer in parallel when the plan allows.
   - Tests must pass before a task is done. No skipped or fake tests.

## Local Contracts (loaded via rules)

All modes must load and follow these shared contracts:

- `.kilo/rules/rdd-core.md` — Workflow order, requirement drafting rules, testing discipline, gates, parallelization, caveman default, subagent usage.
- `.kilo/rules/specref-trace.md` — Tagging placement, source/test coverage, trace integrity, resolution table for common issues.

## Agent Usage

Use the appropriate RDD agent for each mode. All agents are in `.kilo/agents/`:

| Agent | Purpose | Edit Access | How to Invoke |
|-------|---------|-------------|---------------|
| rdd-planner | Plan features, draft requirements, create SpecRef entries | No | `@rdd-planner` or select from agent picker |
| rdd-reviewer | Audit code across 7 dimensions | No | `@rdd-reviewer` or select from agent picker |
| rdd-agent | Debug, root-cause, behavior adjustments | Yes | `@rdd-agent` or select from agent picker |
| rdd-bootstrap | Context gathering helper | No | Invoked via Task tool from other agents |
| rdd-requirements | Requirement analysis helper | No | Invoked via Task tool from other agents |
| rdd-test-writer | Requirement test helper — writes and runs tests for specified R-ids | Yes (tests only) | Invoked via Task tool from other agents |

## Mode Dispatch

On invocation, immediately:

- Load caveman full.
- Classify mode (default: plan).
- Scaffold todo with the canonical phases for that mode.
- Read the local contracts above.
- Check for SpecRef MCP availability (graceful fallback if absent).
- State the mode + "High Signal + Caveman full active" to the user.

### Plan Mode (default)

Phases (use todo_write for each):

0. Bootstrap — Read AGENTS.md + relevant files in scope + contracts + check SpecRef MCP.
1. Frame — Restate problem, constraints, success criteria, risks, non-goals (condensed).
2. Minimal Design — Smallest change fitting existing architecture. Name exact files/modules touched. No new abstractions unless forced.
3. Design Gate — Output the design (high signal only). Then `question` short approval. Stop until explicit approval (or override).
4. Draft Requirements — Use contracts + SpecRef MCP (if available) for dupe/anchor/tier checks. Atomic "shall" statements only. No default 3-tier creation.
5. Requirements Gate — Output the drafted requirements (condensed). `question`. Stop for approval.
6. Write to SpecRef (or local) — Create/update only after approval. Verify traces. Produce R-id list with short summaries.
7. Implementation Plan — Condensed 7a/7b/7c style:
   - File change index (CREATE/MODIFY + one-sentence summary per file).
   - Ordered tasks with R-ids, exact changes, dependencies, and test targets.
   - Requirement test plan with timing per task (after code or parallel).
     Omit any empty categories.

End with crisp "Next Steps" (implement, validate with requirement tests, tag with R-ids).

### Review Mode

Use 7-dimension lens on provided scope/files/diff.

- Run in waves if large scope (Wave 1: docs/deadcode/tagging/structure; Wave 2: tests/trace/missing-reqs).
- **Only emit dimensions that have real findings.**
- Lead with: "N blockers, M warnings across X dimensions."
- List findings with file:line, description, suggested fix, severity.
- Strong focus on R-id tagging hygiene and missing test coverage for implemented requirements.
- Use `specref_*` MCP tools for trace evidence instead of plain grep where possible.

No full 7-row summary table unless user asks for "full report".

### Implement Mode (debug or behavior change)

Classify:

- Debugging: existing behavior is wrong/broken/flaky.
- Behavior Adjustment: intentional change that alters current behavior.

For both:

- Phase 0: Frame symptom + expected behavior.
- Research current reality (read files + contracts + any existing R-ids).
- Scientific method for defects: narrow reproduction → hypotheses → experiments → root cause at code level.
- For behavior change: minimal design preserving architecture + requirement impact check.
- Tests that validate the defect fix or new contract. Write them after the code by default; use rdd-test-writer in parallel when stable.
- Smallest change → green.
- Run targeted regression.
- After material changes, hand off to review mode (spawn subagent or instruct user to invoke rdd-reviewer).

Always surface the confirmed root cause or behavior delta + requirement impact.

### Help Mode

Delegate to the `rdd-help` skill or output the quick reference.

## Subagent Usage

For heavy phases (evidence gathering, multi-wave review, implementation spikes):

- Use the Task tool to invoke subagents.
- Invoke rdd-test-writer for parallel requirement tests; it edits test files only.
- Always prepend the High Signal Rule + caveman activation + relevant contract excerpts.
- Use read-only capability for pure research.
- Parent skill owns todo, gates, and final synthesis.

## Execution Notes

- Every RDD session starts by stating mode + "High Signal + Caveman full active".
- After gates or major phases: crisp status + next action + updated todo.
- End of any mode: minimal "Next Steps" with concrete actions and R-ids.
- Respect the helpful gate philosophy. Note overrides.

## Quick Reference

Invoke RDD via agents:
- `@rdd-planner <description>` — Plan a feature
- `@rdd-reviewer <scope>` — Review code
- `@rdd-agent <problem>` — Debug or adjust behavior
- `@rdd help` — Show quick reference
