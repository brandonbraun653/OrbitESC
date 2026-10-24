---
name: prisma-migration
description: >
  Prisma schema change workflow: design → migrate → generate → verify → test → document.
  Use when making database schema changes, creating migrations, or optimizing queries.
---

# Prisma Migration Skill

Systematic workflow for Prisma schema changes.

## Workflow

### 1. Design

- Read current schema (`packages/db/prisma/schema.prisma`).
- Identify affected models, fields, relations, indexes.
- Check for breaking changes: field type changes, relation deletions, required field additions.
- Propose minimal schema change with rationale.

### 2. Migrate

```bash
pnpm --filter @budget-butler/db prisma migrate dev --name <migration-name>
```

- Migration name: descriptive, kebab-case (e.g., `add-user-preferences`, `fix-transaction-index`).
- Review generated SQL before confirming.
- Verify migration applies cleanly — no manual SQL edits unless justified.

### 3. Generate

```bash
pnpm --filter @budget-butler/db db:generate
```

- Never edit `packages/db/generated/` directly.
- Verify generated types match expected interface changes.

### 4. Verify

- Check TypeScript compiles: `pnpm --filter @budget-butler/web exec tsc --noEmit && pnpm --filter @budget-butler/api exec tsc --noEmit`
- Run unit tests: `pnpm test`
- Run e2e tests if UI flows affected: `pnpm test:e2e:compose` (compose stack must be up).

### 5. Document

- Migration commit message: `feat(db): <description>` or `fix(db): <description>`.
- Include migration purpose and rollback plan in commit body if non-trivial.

## Rollback Strategy

- Prefer reversible migrations (add column before removing old one).
- If migration is destructive, document manual rollback SQL.
- Test rollback against development database before merge.

## Common Patterns

### Adding a field

```prisma
model User {
  id        String   @id @default(cuid())
  newField  String?  // Nullable first, backfill, then make required
}
```

### Adding an index

```prisma
model Transaction {
  id     String @id @default(cuid())
  date   DateTime
  amount Decimal

  @@index([date])
}
```

### Adding a relation

```prisma
model Budget {
  id           String        @id @default(cuid())
  transactions Transaction[]
}

model Transaction {
  id       String  @id @default(cuid())
  budgetId String?
  budget   Budget? @relation(fields: [budgetId], references: [id])
}
```

## Safety Rules

- No raw SQL unless Prisma cannot express the operation.
- Always use transactions for multi-step data migrations.
- Backfill data in batches for large tables.
- Never drop a column without first confirming no code references it.
