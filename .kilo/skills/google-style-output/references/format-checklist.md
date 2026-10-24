# Format and grammar checklist

Quick-reference supplement to `SKILL.md`, derived from the Google developer documentation style guide.

## Punctuation

- **Serial (Oxford) comma**: always before the final `and`/`or` in a list of three or more ("zones, regions, and multi-regions").
- **Commas after introductory words/phrases**: "Finally, only groups…", "Based on the requirements, you can…".
- **Two independent clauses joined by a conjunction** (and/but/or/nor/for/so/yet): comma before the conjunction unless both clauses are very short ("Type your ID and click **OK**").
- **Nonrestrictive `which` clauses**: comma before _which_ ("the group, which has a 200-character limit, …"). Use _that_ for restrictive clauses without a comma.
- **Conjunctive adverbs** (however/therefore/otherwise): semicolon or period before, comma after ("The value must be set; otherwise, the server returns an error").
- **Don't** use a comma before _because_ unless it starts a nonrestrictive clause.
- **Ampersand**: don't use `&` for _and_ in prose, headings, or navigation (except UI labels or space-constrained table/diagram labels, or code).
- **Ellipses**: use for omitted quoted material; avoid trailing "…" to imply continuation in normal prose.
- **Exclamation points**: avoid. **Em dashes**: used thoughtfully, not as commas; don't use a dash to set off a description-list item (use a colon).
- **Parentheses**: minimal; prefer a sentence or a clause. **Slashes**: avoid _and/or_ except in tables; don't use slashes to combine words in prose.
- **Periods**: end every sentence. No period after a title/heading. Decimal points use a period; thousands separators use commas per American English.

## Capitalization

- **Sentence case** for titles, headings, list items, table cells, captions, figure labels, glossary terms, and index terms.
- Capitalize product names as officially branded; refer to them in sentence case when embedded in a sentence.
- After a colon, start lowercase unless the text is a proper noun, a heading, a quotation, or follows a _Note_/_Caution_ label.
- Hyphenated first word in a heading: capitalize only the first element (unless a later element is a proper noun).
- Don't rely on capitalization alone to convey meaning (e.g. `Pod` vs `pod`). Don't use all-caps for emphasis. Avoid camelCase/snakeCase in prose; describe the rule and give an example instead.

## Numbers, dates, and units

- Spell out zero through nine in prose when not technical; use numerals for 10+ and for measurements, counts, versions, ports, and specs.
- Dates: unambiguous, preferably ISO-like or fully written ("2026-07-19" or "July 19, 2026"); avoid ambiguous numeric-only forms like "07/19/26". Use 24-hour or AM/PM with a space ("9:00 AM").
- Units: use standard abbreviations with a space between number and unit ("10 MB"); define any uncommon unit on first use.
- Versions: "version 2.2 or earlier" (not _lower_); "version 3.0 or later" (not _higher_).

## Abbreviations

- Define an abbreviation on first use, then use it consistently ("software development kit (SDK)").
- Don't start a heading with an abbreviation unless it's the widely known form; if used, define it in the first paragraph.
- Standard American spellings favored by the guide: _canceled_, _email_ (not e-mail), _ecommerce_, _backend_, _codebase_, _data center_, _username_.

## Reference documents and API text

- Reference docs state facts in third person about programming elements; address the reader as _you_ only for instructions.
- Method names: omit the class unless needed for clarity ("call its `get` method", not "`animal.get`").
- HTTP status codes: "an HTTP `400 Bad Request` status code" (not _response code_/_error code_); ranges as "an HTTP `2xx` or `400` status code" or "in the `200`–`299` range". Put numbers and names in code font.
- Placeholders: ALL CAPS in code font (`PROJECT_ID`); explain each placeholder near where it appears.

## Notices and emphasis

- Use **bold** only for UI elements and run-in headings, and at the start of notices. Use _italics_ sparingly — for terms-as-words, book/movie titles, mathematical variables, and true semantic emphasis. Never underline except for links.
- Notes, cautions, warnings: lead with the label, keep the body in sentence case, and end with a period.

## Accessibility specifics

- Don't use _above_/_below_/_right-hand side_ to locate document or UI content; use _earlier_/_following_ or describe by name. Provide a screenshot for hard-to-find UI.
- Each image needs meaningful alt text. Offer high-resolution or vector images when practical.
