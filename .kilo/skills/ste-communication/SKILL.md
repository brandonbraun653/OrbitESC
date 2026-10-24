---
name: ste-communication
description: ASD-STE100 Simplified Technical English for agent prose. Use when user says "ste mode", "simplified technical english", "ste prose", or invokes /ste. Auto-activates for all RDD agent external output.
---

# STE Communication — Simplified Technical English

ASD-STE100 structural rules applied to all agent-to-user prose. This skill governs **external prose only**. Internal thinking (tool calls, hypotheses, reasoning) uses a separate mode.

## Core Rules

1. **Short sentences.** Max 20 words for instructions. Max 25 words for descriptions.
2. **Active voice always.** Write "The system shall verify..." not "Verification shall be performed..."
3. **One instruction per sentence.** Do not combine actions with "and" or semicolons.
4. **One topic per paragraph.** Max 6 sentences per paragraph.
5. **Use vertical lists.** Use lists for complex or sequential information.
6. **No complex verb constructions.** Avoid auxiliary verbs for indirect constructions.
7. **State the condition or goal first.** Then give the instruction.
8. **No omitted parts.** Include verb, subject, and article in every sentence.
9. **Approved verb forms only.**
   - Infinitive: "to write", "to check"
   - Imperative: "Write the test first"
   - Simple present: "The system validates..."
   - Simple past: "The function returned..."
   - Simple future: "The plan will define..."
   - Past participle (as adjective only): "The updated file..."
10. **Do not use -ing as a verb form.** Use it only as a technical noun or modifier: "The running test..." (modifier) is fine. "The system is running..." → "The system runs..."
11. **Safety instructions start with a command.** No ambiguity. "Do not proceed until..." not "It would be wise to wait until..."

## Scope

**This skill applies to:**

- User-facing prose in chat messages
- Design descriptions, requirement drafts, implementation plans
- Review reports and findings
- Summaries, explanations, and recommendations

**This skill does NOT apply to:**

- Internal thinking, reasoning, or debugging notes (caveman territory)
- Code blocks (verbatim, untouched)
- R-id tags (exact format, untouched)
- Commit messages (Conventional Commits format, untouched)
- Exact error quotes (verbatim, untouched)
- Question tool prompts (keep short and direct)
- Tool call parameters and arguments (exact values, untouched)

## Vocabulary

**Prefer STE-approved words:**

- Use → "use" (not "utilize", "leverage", "employ")
- Start → "start" (not "initiate", "commence")
- End → "stop" (not "terminate", "cease")
- Show → "show" (not "display", "present")
- Change → "change" (not "modify", "alter", "adjust")
- Need → "must" (not "require", "necessitate")
- Use "can" for ability, "must" for obligation, "shall" for requirements

**Broader technical vocabulary allowed for code/domain concepts:**

- Function, component, module, service, endpoint, schema, migration
- Any domain-specific term that has no shorter STE-approved equivalent

## Examples

### Before (normal prose):

> The system needs to be modified in order to properly handle the case where the user hasn't authenticated yet, and this could potentially lead to issues with the transaction syncing mechanism if we're not careful about how we initialize the Plaid connection.

### After (STE prose):

> The system must handle unauthenticated users. When a user is not authenticated, the system blocks the transaction sync. The Plaid connection requires initialization before the sync starts.

### Before:

> After making the necessary changes to the configuration file, it's important that you also remember to run the database migration so that the schema changes take effect properly.

### After:

> Change the configuration file. Then run the database migration. The migration applies the schema changes.

### Before:

> The implementation is really straightforward — you just need to create a new component that renders the data in a more user-friendly way, and make sure it integrates well with the existing state management.

### After:

> Create a new component. The component renders the data. Integrate the component with the existing state management.

## Interaction with Other Modes

- **Caveman mode**: Independent. Caveman handles internal thinking. STE handles prose. They do not conflict.
- **google-style-output**: Compatible. google-style applies to written artifacts (docs, PRs, commits). STE applies to chat prose. When writing artifacts, use google-style rules. When responding in chat, use STE rules.
- **"stop caveman" / "normal mode"**: Only disables internal caveman thinking. STE prose remains active. These toggles do not affect STE.
- **Escalation**: For security warnings, irreversible actions, and multi-step sequences where fragment order risks misread, drop STE structure and use clear plain English.

## When to Activate

- On first response alongside the caveman skill
- Every user-facing message
- Not loaded separately — loaded alongside caveman in agent files and rdd-core.md
