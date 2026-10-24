---
name: docker-compose
description: >
  Docker Compose service management: status checks, log inspection, health monitoring,
  restart patterns. Use when debugging compose services, checking health, or managing
  the dev stack.
---

# Docker Compose Skill

Service management for the BudgetButler Docker Compose stack.

## Dev Stack

The dev stack uses `docker-compose.dev.yml` with `.env.dev`:

```bash
pnpm dev:compose          # Start all services (background)
```

Services: `postgres`, `api`, `web`, `e2e` (Playwright runner).

## Status Checks

```bash
# All services
docker compose --env-file .env.dev -f docker-compose.dev.yml ps

# Specific service
docker compose --env-file .env.dev -f docker-compose.dev.yml ps web

# Health status
docker compose --env-file .env.dev -f docker-compose.dev.yml ps --format json
```

## Logs

```bash
# All services, tail
docker compose --env-file .env.dev -f docker-compose.dev.yml logs -f

# Specific service
docker compose --env-file .env.dev -f docker-compose.dev.yml logs -f web

# Last N lines
docker compose --env-file .env.dev -f docker-compose.dev.yml logs --tail 50 api
```

## Health Checks

- `web`: http://localhost:3000 (internal), http://localhost:10502 (host)
- `api`: http://localhost:3001 (internal), http://localhost:10501 (host)
- `postgres`: tcp 5432

Check health:
```bash
curl -s http://localhost:10502 > /dev/null && echo "web: healthy" || echo "web: unhealthy"
curl -s http://localhost:10501/api/health && echo "api: healthy" || echo "api: unhealthy"
```

## Restart Patterns

```bash
# Restart specific service
docker compose --env-file .env.dev -f docker-compose.dev.yml restart web

# Rebuild and restart (after dependency changes)
docker compose --env-file .env.dev -f docker-compose.dev.yml up -d --build web

# Full restart
docker compose --env-file .env.dev -f docker-compose.dev.yml down && pnpm dev:compose
```

## E2E Tests

The `e2e` service uses `mcr.microsoft.com/playwright:v1.59.1-noble` with bind-mounted `node_modules`.

```bash
# Run full suite
pnpm test:e2e:compose

# Run single file
docker compose --env-file .env.dev -f docker-compose.dev.yml exec e2e \
  pnpm --filter @budget-butler/web exec playwright test e2e/<file>.spec.ts
```

## Troubleshooting

- **Service won't start**: check `docker compose --env-file .env.dev -f docker-compose.dev.yml logs <service>`
- **Port conflict**: ensure no other process uses 10501, 10502, or 5432
- **Stale containers**: `docker compose --env-file .env.dev -f docker-compose.dev.yml down -v` (removes volumes)
- **e2e browser mismatch**: verify Playwright image tag matches `@playwright/test` version in `apps/web/package.json`
