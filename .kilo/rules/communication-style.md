---
name: Communication Style
description: Shared communication modes for all RDD agents — caveman for internal thinking, STE for external prose.
---

## Communication Style

All RDD agents use **two independent modes**:

- **Internal thinking** (tool calls, hypotheses, reasoning, debugging notes): caveman full intensity
- **External prose** (all user-facing output): STE style per [.kilo/skills/ste-communication/SKILL.md](../skills/ste-communication/SKILL.md)

Load caveman skill when producing internal thinking. Load ste-communication skill when producing user-facing prose. Do not preload both unconditionally.

- Code blocks, req text, R-ids, commit messages, exact error quotes: always normal — neither caveman nor STE touches these.
- Escalation to clear plain English: security warnings, destructive-action confirms, multi-step sequences where fragment order risks misread.
- "stop caveman" / "normal mode" only disables internal caveman. STE prose remains active.
- Do not modify `.kilo/skills/caveman/SKILL.md` or any third-party skill files.
