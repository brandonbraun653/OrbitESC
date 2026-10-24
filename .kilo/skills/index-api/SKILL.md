---
name: index-api
description: Teaches an agent how to use the Index API — a self-hosted PDF memory service for upload, async ingestion, semantic search, and grounded document reading. This skill should be used when an agent needs to store a PDF in Index, poll ingestion status, search ingested documents, read parsed document content, manage Index documents, or connect to Index at https://index.ndandclicky.com/api.
---

# Index API

Index is a self-hosted memory and knowledge service. Upload a PDF; Index parses it, embeds the chunks, and serves semantic search over them. All data stays on the host.

Set the base URL once, then use it in every example below:

```bash
BASE=https://index.ndandclicky.com/api
```

Local development: `BASE=http://localhost:8100/api` (single origin — the console SPA shares the same port at `/`). All REST routes live under `/api`. `GET /llms.txt` and `GET /agent-guide` are also served at the origin root.

Every data route requires an API key. Create one in the console **Account** tab
(administrator sign-in required) and send it as a bearer token:

```bash
export INDEX_API_KEY=idx_...
```

## Before use

1. Check liveness: `curl -s $BASE/health` returns `{"status": "ok"}`.
2. If the service is unreachable, the stack runs on the Index host via Docker Compose from the Index repo root: `docker compose up -d --build`. The first start downloads models into the `model_cache` volume; later starts run offline.

## Live reference

Fetch `GET /agent-guide` (same markdown at `/llms.txt`) for the current workflow, status meanings, and error contract. Fetch `GET /openapi.json` for machine-readable request/response schemas. This skill summarizes the current MVP surface; the live guide is authoritative when they differ.

## Canonical workflow

1. Upload — `POST /documents`, multipart form field `file`:

   ```bash
   curl -s -X POST $BASE/documents \
     -H "Authorization: Bearer $INDEX_API_KEY" \
     -F "file=@report.pdf;type=application/pdf"
   ```

   Returns `202` with `{id, filename, status: "pending"}`. Save the `id`.

2. Poll — `GET /documents/{id}` until `status` is `ready` or `failed`:

   ```bash
   curl -s -H "Authorization: Bearer $INDEX_API_KEY" $BASE/documents/$ID
   ```

   | Status | Meaning | Allowed operations |
   |---|---|---|
   | `pending` | Queued, not started | inspect, list |
   | `processing` | Parsing + embedding | inspect, list |
   | `ready` | Chunks searchable | search, content, download, delete |
   | `failed` | Ingestion failed; `error` field holds the message; original PDF preserved | inspect error, delete, re-upload |

   Poll every 1–2 seconds. Small PDFs finish in seconds. Ingestion runs one document at a time; later uploads queue behind earlier ones.

3. Search — `POST /search`:

   ```bash
   curl -s -X POST $BASE/search \
     -H "Authorization: Bearer $INDEX_API_KEY" \
     -H "Content-Type: application/json" \
     -d '{"query": "how does X work", "document_ids": ["'$ID'"], "top_k": 5}'
   ```

   - `query` is required. `document_ids` (array of ids) is optional and restricts search to specific PDFs — the "answer from this document" workflow. `top_k` is optional; default 5.
   - Response: `[{document_id, filename, chunk_text, score}]`, score is cosine similarity, sorted descending.
   - `404` — an id in `document_ids` does not exist. `409` — an id is not `ready`. `422` — empty query.

4. Read the parsed source — `GET /documents/{id}/content` returns the ingested Markdown. Quote it verbatim when answering; cite `filename` and `document_id`. `409` while pending/processing; `502` with the ingestion error text when failed. Figures appear as `![picture p.2](/api/documents/{id}/assets/image_000000_<hash>.png)` — the alt text holds the label and source page. The href is root-relative to the API host; request it as `$BASE/documents/{id}/assets/{name}`. Documents ingested before the `/api` cutover keep legacy root-relative hrefs (`](/documents/{id}/assets/...)`) that are not rewritten at serve time — resolve those against `$BASE` the same way.

   Fetch a figure with `GET /documents/{id}/assets/{name}` → `200` `image/png`; `404` for an unknown document, unknown name, or a name outside the `image_NNNNNN_<hash>.png` allowlist. No status gating.

5. Fetch the original bytes — `GET /documents/{id}/download` streams the original PDF.

6. Delete — `DELETE /documents/{id}` removes the stored object, extracted figure assets, all chunks and embeddings, and the record. Returns `200` with `{id, deleted: true}`. Returns `409` while the document is `processing`.

## Gotchas

- PDFs only. The check is magic bytes (`%PDF-`), not the declared content type. Non-PDF content returns `415`; an empty or unusable filename also returns `415`.
- Every data route requires `Authorization: Bearer $INDEX_API_KEY`; a missing, revoked, or expired key returns `401`. Administrator routes reject API keys with `403`.
- Filenames are sanitized to the basename. Duplicate filenames are allowed; every upload gets a fresh `id`. No dedup, no project scoping in this version.
- Do not assume `ready` right after upload. Search, content, and delete return `409` until ingestion completes. Read the response `detail` — it carries the status or the ingestion error.
- A `failed` document keeps its original PDF in storage. Content returns `502` with the error text. Inspect `error` on `GET /documents/{id}`, then delete or re-upload.
- `GET /documents` lists every document as `{id, filename, status, size, page_count, uploaded_at, error}`. Use it to find ids when the caller did not supply one.
