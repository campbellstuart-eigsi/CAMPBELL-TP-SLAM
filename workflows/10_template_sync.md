# Workflow 10: Project Template Sync

Use this to compare a project with the master development template and, when I ask, add the universal documentation and workflow files it is missing. It is normally run through the global `/project-sync` skill (`audit`, `apply`, `status`), or when I say **"Run the template sync workflow."**

This file is the canonical procedure. The skill only locates the template and points here.

## Ground rules
- Work only inside the current project folder. Never modify any other project, and never modify the master template while syncing.
- If the current project folder **is** the master template, stop and say so.
- `audit` and `status` are read-only. They never create, change or delete anything.
- `apply` only **adds missing files and folders** from the "safe to add" rows below. It never overwrites, moves, renames or deletes anything.
- Never change an existing file (including `AGENTS.md`, `CLAUDE.md`, `docs/STATUS.md` and existing workflows) without showing the proposed change and getting my explicit approval for that file in this conversation.
- Never touch source code, imports, dependencies, build files, settings or anything else that affects how the application behaves. Only the paths listed in the file table are in scope.
- Never stage, commit or push. Never sync automatically at session start; run only when I ask.
- Never invent information. Placeholders copied from the template stay as placeholders (`<TBD>` or `<...>`).

## Step 1: Locate the master template
1. If the environment variable `PROJECT_TEMPLATE_DIR` is set, use that folder.
2. Otherwise use the default: `C:\Users\stuar\Documents\DEVELOPMENT_TEMPLATE`.
3. The folder is valid only if it contains both `TEMPLATE_VERSION` and `workflows/10_template_sync.md`.
4. If it is not valid, stop and report: **"Master template not found"**, the paths you checked, and how to fix it (set `PROJECT_TEMPLATE_DIR` or restore the default folder). Make no changes.
5. If the template cannot be read because reads outside the working directory are blocked, report **"Master template not readable from this project"** and ask me to add the folder (`/add-dir <template path>`, or start Claude with `--add-dir <template path>`). Make no changes.

The template version is the first line of `<template>/TEMPLATE_VERSION`.

## Step 2: Identify the project
- The project root is the current working directory of the session.
- Read the project's `.template-version` if it exists (the template version last applied). If it is missing, the project has never been synced.

## Step 3: Compare, file by file
Compare each row below. When comparing file contents, ignore differences that are only line endings (CRLF vs LF) or a trailing newline.

| Template path | Kind | If missing in project | If present in project |
|---------------|------|-----------------------|-----------------------|
| `workflows/*.md` (every file in the template's `workflows/`) | Universal | **Safe to add**: copy it unchanged | Identical → OK. Different → report **Differs**; show a diff only if I ask; replace only with my approval |
| `docs/STATUS.md` | Universal skeleton | **Safe to add**: copy the template skeleton | Never replace (it holds project state). Report template section headings (`## ...`) it lacks |
| `docs/decisions/`, `docs/design/`, `docs/research/`, `docs/specs/` | Universal folders | **Safe to add**: create the folder with an empty `.gitkeep` | Folder exists → OK, even if it has no `.gitkeep` |
| `AGENTS.md` | Project-specific | **Needs approval**: propose creating it from the template | Never replace. Report template section headings (`## ...`) it lacks and propose the text to add |
| `CLAUDE.md` | Project-specific | **Needs approval**: propose creating it from the template | Never replace. Report whether it imports `@AGENTS.md` and `@docs/STATUS.md`; propose any missing import line |
| `TEMPLATE_VERSION` | Template only | Never copied. Its value is recorded in the project's `.template-version` | — |
| `.template-version` (project only) | Sync record | **Safe to add** during `apply`: one line, the template version | Same version → OK. Different → **Needs approval** to update the version line |

Anything in the project that is not in this table is out of scope: do not report on it or change it. Files that only the project has (for example `docs/SESSION_HANDOFF.md`, extra workflows, specs) are left alone.

Sync status, for the report:
- **In sync**: nothing missing, nothing differs, `.template-version` matches.
- **Safe additions available**: only "Safe to add" items are outstanding.
- **Needs review**: something differs or needs approval.

## Step 4: Run the requested mode

### audit (default)
Read-only. Report:
1. Template path and version; project path and recorded version (or "never synced").
2. A table: path, result (`OK`, `Missing – safe to add`, `Missing – needs approval`, `Differs`, `Sections missing: ...`).
3. What `apply` would add, and what would need my approval.
4. The overall sync status.

### status
Read-only and short: template version, project's recorded version, overall sync status, and counts of OK / safe-to-add / needs-review items, with the needs-review paths listed.

### apply
1. Run the audit first and show its results.
2. Add every "Safe to add" item. Copy template files unchanged; do not fill placeholders.
3. Write `.template-version` only if it is missing.
4. Do **not** change any existing file. For each "Needs approval" or "Differs" item, list the proposed change and ask me. Act only on the files I approve, one at a time, showing what changed.
5. Re-run the comparison and report: files added, files left for approval, and the new sync status.

Running `apply` again on an unchanged project must add nothing and change nothing.

## Step 5: Report
End every run with:
1. Mode run, template path and version, project path.
2. Files created (paths), or "None". Files changed with approval, or "None".
3. Items still needing review or approval.
4. Overall sync status.
5. A reminder that nothing was staged, committed or pushed.

Then stop. Do not start other work.
