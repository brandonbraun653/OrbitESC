---
name: RDD Core
description: Shared Requirement Driven Development workflow and requirement-writing rules for SpecRef agents.
---

Shared RDD semantics. This file is the canonical source for the RDD workflow.

## Workflow Order

- Follow RDD sequence in order: think, minimal design, design buyoff, requirement drafting, requirements buyoff, SpecRef writes, then implementation followed by requirement-validation tests.
- If approved design/reqs already provided, resume from first unfinished step.
- Shared helpers gather evidence only. Parent agent owns approvals, design judgments, final recs.

## Requirement Drafting Rules

- Call SpecRef meta-rule before drafting or evaluating requirement text.
- Check existing reqs + doc hierarchy before drafting — catch dupes early.
- Choose only tiers that fit scope:
  - SRD: top-level capability or mission behavior. No code traces; no up-trace. Traces only down to HLRs.
  - HLR: integration/e2e behavior derived from SRD. Traces up to SRDs, down to LLRs, to system-level tests. Mark `isDerived: true` when too impl-specific for system feature link — waives upstream SRD trace, preserves all evidence obligations.
  - LLR: impl contract, algorithm, validation, error-handling, precision, or timing derived from HLR. Traces up to HLRs, to code declarations + unit tests. Mark `isDerived: true` when too impl-specific — waives upstream HLR trace, preserves all evidence obligations.
- Every public-facing interface must have a dedicated LLR capturing its behavioral contract. Public-facing includes: exported functions, public class methods (NestJS services + controllers), REST controller endpoint methods, exported TypeScript interfaces/types, React component prop interfaces, and Next.js route handler exports. One interface, one LLR — no bundling.
- One behavior per requirement. Split conjunctions.
- Use objective "shall" language with outcomes a test can verify.
- Keep req text brief. Rationale in parent agent response, not req sentence.

## Clarifying Questions

- When you need user input — disambiguation, scope, approval choices, missing context, gate decisions — surface it through the `question` tool. Do not bury the question in prose.
- **Proposals and detailed content belong in the chat message, not in the question tool.** When presenting a design, requirement set, implementation plan, or review report for approval: output the full content as a normal chat message first, then call `question` with only a short gate question (e.g. "Approve this design?"). Never embed multi-line prose, code blocks, or numbered lists inside a question `message` field.
- Use `options` for fixed choices (gate approvals, mode selection, file/scope picks, severity). Mark a `recommended` option when one is clearly preferred. Use `multiSelect: true` only when several answers genuinely apply.
- Keep questions short (one sentence each), give each a unique `header`, and only batch genuinely independent questions in one call. Free-form text is allowed unless a strict choice is required (`allowFreeformInput: false`).
- Never use the questions tool for secrets (passwords, tokens, API keys) — answers are sent to the model. For secrets, instruct the user to provide them out-of-band.
- This rule applies to user-invocable agents (Planner, Reviewer, Rework). Hidden helper agents must not prompt the user; they return open questions to the parent agent instead.
- Gate execution details: see `rdd-planner.md` Steps 3 and 5.

## Parallelization

- Batch independent reads, tool calls, sub-agent invocations.
- No deps → invoke in parallel. Wait for both before proceeding.
- Sequential only when output dep or buyoff gate pending.
- Safe parallel examples: multi-file reads, `specref_get_meta_rule` + `RDD Requirements`, hierarchy fetch + index status.
- The first RDD agent in a session should run full bootstrap. Subsequent agents should reference the cached context from the parent agent's output if no file modifications have occurred since bootstrap.

## Testing Discipline

- Tests SHALL validate requirement outcomes, not implementation details.
- Test timing is chosen per task. Default: write tests after the implementation. Use parallel test writing only when the requirement contract and test surface are stable. The plan records the timing.
- RED-first is not required. Tests MUST pass before a task is marked done.
- HLR: proven by integration/e2e tests. LLR: proven by unit tests.
- Every implemented R-id MUST have at least one test asserting its behavior, with the R-id tag above the `it`/`test` block.
- Tests MUST cover the failure and edge paths named by the requirement.
- Targeted tests > broad suites.
- Do not mark work complete with missing, skipped, failing, or fake tests.
- If you cannot run tests (environment block), stop and explain. Do not proceed without verification.
- Full test execution contract in `rdd-planner.md` Step 7d.

## Document Restructuring

When reorganizing existing SpecRef doc (add headers, group reqs, restructure parent/child) without changing req text or traces:

**Phase 0 — Drift check.** Call `specref_list_requirements`. Follow `nextSiblingRId` chain from root's `firstChildRId` to build sibling order. Confirm matches expected. Anything off → stop, report drift before writing.

**Phase 1 — Create headers.** Call `specref_create_requirement` with `isRequirement: false`, `parentId` = root ID, H2 text (`## Section Name`). Create sequentially in display order — appends after existing children.

**Phase 2 — Move requirements.** Call `specref_move_requirement`. Critical:

- `insertAfterRId: null` **prepends** — makes item FIRST child of parent, not last.
- Forward order: chain `insertAfterRId` to prev moved ID. First item only: `insertAfterRId: null`.
- Move sequentially within each section — sibling order depends on it.
- Append to existing section: find current last child, use its ID as `insertAfterRId` for first new item.

**Phase 3 — Verify.** Call `specref_get_document_hierarchy`. Confirm: (a) root's only direct children = new headers, (b) each section's first/last child + count match plan, (c) sub-hierarchies preserved. Then `specref_get_trace_matrix_status` — moves don't affect traces, pass/fail unchanged.

**Verification.** A pure SpecRef structural reorg has no code tests. Phase 3 verification is the correctness gate.

**Section naming.** Use `## Section Name` (H2). Mirror parent doc section names — LLR aligns with HLR, HLR aligns with SRD. Tier-specific section only when no parent-doc parallel.

## Communication Style

Load and follow `.kilo/rules/communication-style.md`.

## Compaction Recovery

When context is compacted mid-workflow (auto or manual):

1. Use `todoread` to restore current phase state.
2. Re-read plan files if implementing from a plan (`.kilo/plans/`).
3. Re-load active requirements from SpecRef if working with traces.
4. Do not re-do completed work — verify state before continuing.
5. If phase state is ambiguous, ask the user via `question` tool.
