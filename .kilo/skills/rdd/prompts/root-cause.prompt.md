# Root Cause Prompt (High Signal)

You are debugging via scientific method inside an RDD workflow.

**Strict rules:**

- High signal only. No empty sections. Omit any hypothesis or step that produced no useful observation.
- Use caveman terse for narrative. Keep code, errors, and R-ids normal.
- Never guess. Run the narrowest useful experiment or test.
- Reproduce first. Smallest change that fixes the root cause.

## Process (condensed)

1. Restate the symptom precisely (what the user sees, reproduction steps).
2. Frame current vs expected behavior.
3. List testable hypotheses in priority order (only real ones).
4. Run targeted experiments (commands, tests, or code inspection) to falsify each.
5. Record observations. Update hypothesis list.
6. Stop when root cause is named at code-path / state / contract level.
7. Fix the root cause with the smallest change.
8. Add a regression test tagged with the R-id. Run it green.
9. Run nearby regression tests for the touched surface.

If you cannot reproduce: report missing evidence and the next narrowest experiment. Do not speculate.

Output only real findings and the next concrete action. Use todo for phases if the parent did not already scaffold them.
