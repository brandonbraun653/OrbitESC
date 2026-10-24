---
name: google-style-output
description: Output styling that makes human-facing written artifacts read like Google developer documentation. Enforces conversational tone, second person, active voice, present tense, sentence-case headings, descriptive link text, code-font discipline, and inclusive, global-audience-friendly language. Activate only when the agent writes artifacts that communicate back to a human, such as documentation, docstrings, code comments, commit messages, PR descriptions, changelogs, and release notes. Do NOT apply to ordinary in-chat conversation.
---

# Google style output

Style **human-facing written artifacts** using the [Google developer documentation style guide](https://developers.google.com/style).
This skill is **on-demand**, not always-on: activate it only when producing content that a human will
read as documentation or prose, including:

- Documentation files (markdown, rst, docs sites, READMEs)
- Docstrings, code comments, and inline explanations
- Commit messages, PR descriptions, changelogs, and release notes
- Tutorials, guides, summaries, and other expository writing

**Do NOT apply this skill to ordinary in-chat conversation** — chat replies, tool calls, status
updates, and back-and-forth discussion stay in the agent's normal concise voice. When in doubt about
whether something is human-facing prose, treat it as out of scope.

The guide's core principle: _be clear and consistent for the reader_. When a rule would hurt clarity
for this specific audience, break it deliberately and stay consistent about the break.

Load `references/word-list.md` and `references/format-checklist.md` when a specific word choice or a
formatting edge case comes up; otherwise the rules below are sufficient.

## Voice and tone

- Be conversational, friendly, and respectful — like a knowledgeable colleague. Not slang, not formal, not frivolous.
- Address the reader directly with **you / your** (second person). Avoid _we_, _our_, _us_ for the reader; reserve first-person plural for the org/author ("contact our support").
- Use **active voice** and make the actor explicit: "The server sends a response" not "A response is sent."
- Use **present tense** for general behavior ("The function returns a string"). Use future (_will_) only for genuinely future outcomes, never to describe how a feature "will work after the release."
- Keep it direct. Avoid:
  - Exclamation points and marketing enthusiasm.
  - Buzzwords, jargon, and culturally specific references (sports, pop culture, idioms).
  - Figurative/ableist language, metaphors, and clichés.
  - Placeholder filler like _please note_, _at this time_, _simply_, _easily_, _just_, _obviously_.
  - Phrasing that insults or demeans any group; _let's_ framing.
- Politeness without overuse of _please_. In procedures, don't use _please_ at all.
- Write for a **global audience**: simple, consistent wording that translates well. Avoid idioms and directional/visual-only cues.

## Sentence and paragraph craft

- Put the condition, context, or goal **before** the instruction: "To delete the doc, click **Delete**" — not "Click **Delete** to delete the doc." This lets readers skip what doesn't apply.
- One idea per paragraph. Keep paragraphs to 5–6 sentences max; a one-sentence paragraph is fine. Lead with the key point.
- Prefer short sentences. Don't pad sentences just to shrink paragraph count.
- Left-align text. Never force line breaks or hard returns inside prose.

## Headings and titles

- Use **sentence case** everywhere: capitalize only the first word, the first word after a colon, and proper nouns. No trailing period.
- Task-based heading: start with a **bare infinitive** ("Create an instance"). Conceptual heading: use a **noun phrase** ("Migration to Cloud"). Avoid leading _-ing_ verbs ("Creating an instance" → "Create an instance").
- One unique `h1` per page; never skip heading levels (h1 → h2 → h3). No empty headings, no numbers in headings to imply sequence, no links inside headings.
- Don't repeat the page title as a section heading.

## Lists and procedures

- **Numbered lists** for sequences/steps; **bulleted lists** for non-sequential sets; **description lists** for term/definition pairs.
- Introduce a list with a complete sentence ending in a colon. Use parallel syntax across items.
- Capitalize the first word of each list item. End items with a period unless the item is a single word, has no verb, is all code font, or is a document title.
- Procedures:
  - Number the steps. One action per step.
  - Start each step with an imperative verb ("Click **Submit**"). State the goal before the action when useful ("To start a doc, click **File > New**").
  - Set context (tool/UI location) before the action: "In the console, go to the **Monitoring** page." Restate context if procedures are split across headings.
  - Use `>` for sequential menu picks: "Click **File > New > Document**."
  - Mark optional steps with "Optional:" at the start (not "(Optional)").
  - Don't include keyboard shortcuts in prose; don't say "press Enter" unless required.
  - Single-step procedures: write as one bulleted sentence, not a 1-item numbered list.
  - Sub-steps: lowercase letters; sub-sub-steps: lowercase Roman numerals.
- When multiple ways exist to do a task, document the shortest, simplest, most accessible one.

## Linking

- Use **descriptive link text** that makes sense standalone. Never "click here", "this document", or a bare URL as text.
- Introduce cross-references consistently: "For more information, see [Title]." Add "about…" when the link purpose isn't obvious. Use _about_, not _on_.
- Put punctuation **outside** link text. Don't put link text in quotation marks.
- Don't force new tabs; if a link must, say so in the text ("…opens in a new tab").
- Keep links selective — each link is a reader decision and a chance to lose them.

## Code-font discipline

- Use backticks (Markdown) or `<code>` for: filenames, paths, class/method/function names, attribute names, command names (`kubectl`, `gcloud`), command output, environment variables, data types, enum names, HTTP verbs and status codes, port numbers, query params, placeholder variables, and text the reader enters verbatim.
- **Do not** code-font: product/service/organization names, domain names, or browser URLs (format URLs as descriptive links instead).
- Sometimes code-font: Boolean literals (`true`/`false`), tool names that match a project name (use code font for the command, plain for the product), email addresses used as input/output.
- UI elements rendered from entered text get **bold + code font** (e.g. **`my-instance`**).
- Don't inflect code names as English (no "POST the data"); pair them with a noun ("send a `POST` request"). Don't wrap code in quotation marks unless the marks are part of the code.

## Word choices (consult references/word-list.md for the full list)

- Prefer **and** over **&** (except in UI labels). Use the **serial comma** (Oxford): "zones, regions, and multi-regions."
- Use **lets you**, not _allows you to_ or _enables you to_.
- Avoid _etc._, _and so on_, _e.g._ (use "for example" / "such as"). Avoid _above_/_below_ for document position (use _earlier_/_following_); avoid directional UI language.
- Use **select** (not _check_) for checkboxes, **clear** (not _uncheck_/_deselect_) to remove a check.
- Use **app** over _application_ (except standard phrases like "application programming interface").
- Replace non-inclusive terms: _allowlist_/_denylist_ over whitelist/blacklist; _blocklist_/_excludelist_; avoid _master/slave_, _dummy_, _sanity_, _blind_, _crippled_, _abort_ (prefer stop/exit/cancel), _blacklist_ verbs rewritten as phrases.
- Avoid _currently_, _as of this writing_, _does not yet_, _eventually_ in timeless documentation.
- Spell standard American English: _canceled_, _email_, _ecommerce_, _backend_, _codebase_, _data center_, _username_, _whitelist→allowlist_.

## Inclusive and accessible language

- Refer to people by their role or identity preference; don't define people by a disability. Use identity-first or person-first as the community prefers ("a Deaf person", "people with disabilities").
- Avoid ableist metaphors and terms like _blind to_, _crippled_, _sanity check_, _dummy_.
- Don't describe nondisabled people as _normal_ or _healthy_.
- Make content navigable without visual-only cues; provide screenshots for hard-to-find UI, not "the button with three lines."

## Self-check before writing the artifact

1. Is this a human-facing artifact (docs, comments, commit/PR text)? If it's in-chat chat, stop — this skill doesn't apply.
2. Second person? Active voice? Present tense for general behavior?
3. Sentence-case headings, no leading _-ing_?
4. Lists parallel, punctuated, introduced with a sentence?
5. Code/UI/products in the right font? No stray `&` or _etc._?
6. Link text descriptive and standalone? Punctuation outside links?
7. No exclamation points, no _please note_, no jargon, no idioms?
8. Clear, direct, global-audience friendly?

When a rule conflicts with clarity for this audience, break it on purpose and stay consistent.
