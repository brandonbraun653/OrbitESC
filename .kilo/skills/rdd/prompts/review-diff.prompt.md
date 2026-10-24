# Review Diff Prompt (High Signal)

Review the current diff (staged, unstaged, or path-provided) using the RDD 7-dimension lens.

**High signal rules:**

- Only report real issues. Omit any dimension with zero findings.
- No template tables with empty rows.
- Lead with a one-line summary: "N blockers, M warnings across X dimensions."
- Then list only actual findings, tagged by dimension inline.
- Caveman terse for descriptions. Exact file:line and symbols in backticks.
- Strong emphasis on traceability, R-id tagging hygiene, and test coverage gaps.

## Dimensions (only surface if findings exist)

1. Documentation — missing JSDoc on public API, unclear comments.
2. Dead code — unused exports, unreachable branches (cross-check with knip or equivalent if available).
3. SpecRef Tagging — misplaced, duplicated, missing, or stale `// R<id>` / `@req R<id>`. Verify existence via MCP if possible.
4. Structure / Simplicity — copy-paste blocks ≥5 lines, verbose expressions, mixed concerns, import order issues.
5. Testing — missing tests, fake/cheap tests (no-op assertions, tautological mocks, self-testing mocks, weak output checks, skipped bodies, swallowed catches, sole `not.toThrow()`).
6. Traceability — source R-id without test coverage, empty up/down traces, orphans, index health problems.
7. Missing Requirements — non-trivial exported or internal behavior with no matching requirement.

For each finding: file:line, short description, concrete fix suggestion, severity (BLOCKER / WARNING / INFO).

End with recommended next actions (highest priority first). No throat-clearing.
