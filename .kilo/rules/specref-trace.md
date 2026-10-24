---
name: SpecRef Traceability
description: Shared SpecRef tagging, traceability, and source-to-test coverage rules for SpecRef agents.
---

Shared SpecRef traceability semantics.

## Tag Placement

- Place R-id immediately above exact implementing declaration, branch, state write, or UI action.
- No broad tag above wrapper when implementation sits deeper — tag narrower location.
- If one branch satisfies requirement, tag that branch, not whole helper.
- No bundling unrelated requirements in one comment block.
- For public-facing interfaces (exported functions, public class methods, REST controller endpoints, exported TypeScript interfaces/types, React component prop interfaces, Next.js route handler exports): place the LLR R-id inside the JSDoc/TSDoc docstring as `@req R<id>`. This docstring tag is the source evidence for that LLR — replaces the `// R<id>` inline comment above the declaration for these cases.

## Tag Format (bare reference only)

- A tag is a bare reference: `// R<id>` (inline) or `@req R<id>` (JSDoc/TSDoc). Nothing else.
- **Do NOT append the requirement text, a description, or a summary after the R-id** (e.g. write `// R42`, never `// R42: The system shall...`). Requirement text lives in SpecRef, not in source comments; duplicating it in code creates drift and bloat.
- The same rule applies to test tags: place a bare `// R<id>` immediately above the `it`/`test` block. No text after the id.

## Source And Test Coverage

- SRDs: no source-side R-ids needed. Coverage satisfied by downstream HLR traces.
- LLR source-side R-id must appear in at least one unit test.
- HLR R-id must appear in at least one system-level (integration or e2e) test.
- Search both source and tests for same R-id before declaring coverage complete.
- Missing test-side coverage for implemented LLR or HLR R-id is blocker, not nice-to-have.

## Trace Integrity

- After creating or updating requirements, verify record and inspect trace.
- **Move safety.** `specref_move_requirement` changes only `parentId` + sibling-order pointers (`firstChildRId`, `nextSiblingRId`, `parentRId`). No trace link changes. Struct reorg is trace-safe — pass/fail unchanged.
- SRDs: no up-trace, no code traces. Flag SRD missing downstream HLR.
- HLRs: must trace up to SRD, down to LLR, to system-level test. Exception: `isDerived: true` waives upstream SRD trace.
- LLRs: must trace up to HLR, to code declarations, to unit test. Exception: `isDerived: true` waives upstream HLR trace.
- `isDerived` waives upstream trace only — all evidence (downstream LLRs, tests, source tags) still required.
- Flag non-derived HLRs or LLRs with empty up-trace.
- Missing file refs in trace data: check index health before assuming unused.

## Resolving Trace Issues

One correct resolution per `TraceMatrixIssueKind`. Fix root cause, no bandaids.

| Issue kind                          | Root cause                        | Resolution                                                                                                                                                                                                |
| ----------------------------------- | --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SRD_MISSING_DOWNSTREAM_HLR`        | No HLR traces up to SRD.          | Draft HLR deriving from SRD; add up-trace.                                                                                                                                                                |
| `HLR_MISSING_UPSTREAM_SRD`          | HLR has no up-trace to any SRD.   | Add up-trace to existing SRD, or draft new SRD. If behavior too impl-specific for system feature link but worth capturing, use `isDerived: true` — waives upstream trace, keeps all evidence obligations. |
| `HLR_MISSING_DOWNSTREAM_LLR`        | No LLR traces up to HLR.          | Draft LLR narrowing HLR into impl contract; add up-trace.                                                                                                                                                 |
| `HLR_MISSING_VERIFICATION_EVIDENCE` | No integration/e2e test evidence. | Write integration or e2e test; place R-id comment above `it`/`test` block.                                                                                                                                |
| `LLR_MISSING_UPSTREAM_HLR`          | LLR has no up-trace to any HLR.   | Add up-trace to existing HLR, or draft new HLR. If behavior too impl-specific, use `isDerived: true` — waives upstream trace, keeps all evidence obligations.                                             |
| `LLR_MISSING_SOURCE_EVIDENCE`       | R-id absent from source files.    | Place a bare `// R<id>` above exact implementing declaration, branch, or state write. No requirement text after the id.                                                                                   |
| `LLR_MISSING_UNIT_TEST_EVIDENCE`    | No unit test evidence.            | Write unit test; place a bare `// R<id>` above `it`/`test` block. No text after the id.                                                                                                                  |

**Resolution discipline:**

- Default: fix evidence gap. Do not delete requirement.
- Delete only when genuinely obsolete (feature removed, behavior gone, exact dupe). Surface finding + reasoning. Explicit user confirm required.
- Re-run trace matrix after fix to confirm issue disappears.
- HLR gap (test) and LLR gap (source tag + unit test) are independent — both must be fixed when both present.

## Review Expectations

- Missing req linkage for non-trivial behavior = real defect, not doc polish.
- Surface stale tags, deleted reqs, dupe tags on unrelated logic, trace gaps.
- Parent agents own final blocker semantics + user-facing reports. Helpers collect evidence only.
