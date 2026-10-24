# Word list (developer-communication subset)

Curated from the Google developer documentation style guide word list. Apply when choosing words in
prose, instructions, commit messages, and reviews. Full list: https://developers.google.com/style/word-list

Legend: **avoid** = use with caution / prefer alternative; **don't** = avoid entirely.

## Audience, voice, and tone words

- **don't** _access_ (verb) — use _see_, _view_, _edit_, _find_, _use_.
- **don't** _actionable_ in the legal sense; usually drop it or say "that you can act on".
- **don't** _allows you to_ / _enables you to_ — use _lets you_.
- **don't** _aka_ — write "also known as" or use parentheses/or.
- **don't** _and/or_ — use "and", "or", or rephrase (tables excepted).
- **don't** _as_ for causation — use _because_.
- **avoid** _above_ / _below_ for document or UI position — use _earlier_ / _following_ / _preceding_.
- **avoid** _currently_, _as of this writing_, _does not yet_, _eventually_ — they age badly; state the fact.
- **avoid** _desire_ / _desired_ — use _want_ / _need_.
- **don't** _e.g._ — use "for example" or "such as".
- **don't** _etc._, _and so on_, _and so forth_ — list items or say "including".
- **don't** _exploit_ to mean "use" — only for vulnerabilities.
- **don't** _please_ in procedures; minimize elsewhere.

## Checkboxes and UI

- **don't** _check_ (a checkbox) — use **select**.
- **don't** _uncheck_ / _deselect_ (clear a checkbox) — use **clear**.
- **don't** _click on_ — use _click_ (hyphenate _right-click_, _double-click_). On Android use _tap_, not _click_.
- **don't** call a link a _button_; don't call a mechanical control by the wrong term.
- **don't** _dropdown_ as a standalone noun; prefer _list_ / _menu_; _drop-down_ only as a modifier if needed.
- **don't** _choose_ for UI elements — use **select** (generic _choose_ is fine).

## Computing and code terms

- Spellings: **backend** (not back-end/back end), **codebase**, **data center**, **data source**, **datastore**,
  **data type**, **email** (not e-mail), **ecommerce** (not e-commerce), **endpoint** (not end point),
  **username** (not account name), **whitelist→allowlist** / **blacklist→denylist/blocklist**.
- **don't** _blacklist_ / _whitelist_ / _graylist_ (nouns or verbs) — use _allowlist_, _denylist_, _blocklist_,
  _excludelist_, _safelist_; for verbs, rewrite the phrase (e.g. "deny requests from an IP" not "denylist an IP").
- **don't** _master/slave_ phrasing — use precise terms (_primary/replica_, _leader/follower_).
- **avoid** _dummy_ (use _placeholder_), _sanity check_ (use _quick check_ / _smoke test_),
  _blind_ to mean ignore (use _ignore_ / _unaware of_), _crippled_ (use _slowed_ / _limited_),
  _abort_ in general prose (use _stop_ / _exit_ / _cancel_).
- **don't** _config_ in prose — use _configuration_ / _configuring_ (keep verbatim code names as-is).
- **don't** _execute_ when _run_ suffices.
- **avoid** _create a new …_ unless distinguishing from a recent item — use _create a …_.
- **don't** _curl_ as _cURL_; write **curl**.
- **don't** _comprise_ — use _consist of_ / _contain_ / _include_.
- **don't** _co-locate_ — use **colocate**.
- **avoid** _canary_ / _cold failover_ / _spaghetti_ jargon without defining on first use.
- **don't** _console_ in isolation — name it ("Google Cloud console"); after first mention, "the console".

## People and inclusivity

- Refer to people by role/identity preference; don't define by disability. Person-first ("person with a disability")
  or identity-first ("Deaf person", "autistic person") per community preference.
- **don't** call nondisabled people _normal_ / _healthy_; use _nondisabled person_, _sighted person_, etc.
- **don't** _abnormal_ / _deficient_ / _deformed_ for people (OK for system conditions).
- **don't** _crazy_ / _bonkers_ / _mad_ / _insane_ / _lunatic_ — use _complex_, _unexpected_, _baffling_.
- **don't** _cripple_, _blind_ (metaphor), _dummy_, _sanity_ — see computing section.
- **don't** _blackhat_ / _black-box_ monitoring — use _synthetic monitoring_; testing → _opaque-box testing_.
- **don't** _brown bag_ — use _learning session_ / _lunch and learn_.

## Conjunctions, transitions, and references

- Use **and** not **&** (UI labels excepted).
- Use **about** (not _on_) in "For more information about…".
- Refer to a group of sections as **the following sections**, not "this section"/"these sections".
- Use **see** for links/cross-references.
- Use **app** over **application** except in standard phrases ("application programming interface").
- Use **later** / **earlier** for version ranges (not _higher_ / _lower_).

## Standard American spellings

canceled (not cancelled), catalog, center, color, email, ecommerce, favorite, fiber, gizmo-free,
initialize, labeled, organize, parameterize, program (not programme), recognize, synchronize,
username, whitespace, writable, dialog (UI) / dialogue (speech), gray (not grey, for UI),
capitalize (not capitalise). First-listed Merriam-Webster spelling wins when unsure.
