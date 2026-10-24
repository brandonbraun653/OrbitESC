---
name: nestjs-controller
description: >
  NestJS controller patterns: decorators, validation pipes, auth guards, error handling,
  Swagger docs. Use when building or reviewing NestJS API endpoints.
---

# NestJS Controller Skill

Patterns and conventions for NestJS controllers in `apps/api`.

## File Structure

```
apps/api/src/
  <module>/
    <module>.module.ts
    <module>.controller.ts
    <module>.service.ts
    <module>.dto.ts
    <module>.entity.ts        (if applicable)
    <module>.spec.ts
```

## Controller Conventions

### Decorators

```typescript
@Controller('transactions')
@ApiTags('transactions')
@UseGuards(AuthGuard)
export class TransactionsController {
  @Get()
  @ApiOperation({ summary: 'List transactions' })
  async findAll(@Query() query: ListTransactionsDto): Promise<Transaction[]> {
    return this.service.findAll(query);
  }
}
```

### Validation Pipes

Always use `ValidationPipe` with DTOs:

```typescript
@Post()
async create(@Body() dto: CreateTransactionDto): Promise<Transaction> {
  return this.service.create(dto);
}
```

DTOs use `class-validator` decorators:

```typescript
export class CreateTransactionDto {
  @IsString()
  @IsNotEmpty()
  accountId: string;

  @IsDecimal({ decimal_digits: '2' })
  amount: string;

  @IsDateString()
  date: string;
}
```

### Auth Guards

- Apply `@UseGuards(AuthGuard)` at controller or route level.
- Extract user from `@Request() req: AuthenticatedRequest`.
- For dev-only routes: gate on `SEED_PROFILE === 'development'`.

### Error Handling

- Let exceptions propagate to NestJS exception filters.
- Use `NotFoundException` for missing resources.
- Use `BadRequestException` for validation failures.
- Never catch and swallow errors silently.

### Swagger Documentation

- Every endpoint needs `@ApiOperation` and `@ApiResponse`.
- Use `@ApiBearerAuth()` for authenticated endpoints.
- DTOs need `@ApiProperty()` for Swagger schema generation.

## Service Layer

- Controllers delegate to services — no business logic in controllers.
- Services handle Prisma queries, validation, and orchestration.
- Inject services via constructor dependency injection.

## Testing

- Unit tests: mock services, test controller method delegation.
- Integration tests: supertest against NestJS app.
- E2e tests: Playwright for user-facing API flows.

```typescript
describe('TransactionsController', () => {
  it('should list transactions', async () => {
    const result = await controller.findAll({ accountId: 'test' });
    expect(result).toBeDefined();
  });
});
```
