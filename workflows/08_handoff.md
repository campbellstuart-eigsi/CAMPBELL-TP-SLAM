# Workflow 08: Session Handoff

Use this before ending a significant development session, or when I say **"Run the session handoff workflow."**

Goal: leave the project so that a new session, with no access to this conversation, can pick up the work from the files alone.

Applies to any project type: software (Python apps, websites, plugins, Linux applications), hardware and electronics, mechanical, and creative projects. Some steps refer to Git, tests or build tools; if the project has none, skip that part and say so in the handoff.

## Ground rules
- Do not start new development work. This workflow only reviews, documents and reports.
- Do not stage, commit, push, delete, move or reorganize project files without my explicit permission.
- Do not run expensive, slow, destructive or hardware-dependent tests (flashing devices, long renders, GPU benchmarks, anything touching real hardware) unless I ask. Cheap checks that are already part of the project are fine.
- Never invent information. Every statement in the handoff is either confirmed (seen in the files, Git or a command run in this session) or marked as an assumption.
- Never claim a test passed unless it ran and you saw the result. Record the exact command and outcome.
- The Project Documentation Workflow rules in `AGENTS.md` still apply: never overwrite accurate documentation with guesses, never delete existing work, never change a spec's `Status:` yourself.

## Step 1: Review the project
1. Read `AGENTS.md`, `docs/STATUS.md` and any existing `docs/SESSION_HANDOFF.md`.
2. Skim the rest of the relevant documentation: `docs/specs/`, `docs/decisions/`, `docs/design/`, `docs/research/`, the README and any project-specific docs.
3. Inspect the actual project files, not just the docs. Check that what the docs say exists really exists.
4. If the folder is a Git repository, run `git status`, `git diff --stat` and `git log --oneline -10`. If it is not, note "No Git repository" and, where useful, list recently modified files instead.
5. Sort the work into four groups:
   - **Complete**: done and verified.
   - **Incomplete**: started but not finished (say where).
   - **Planned**: agreed or specified but not started.
   - **Unverified**: written or changed but not tested or checked.

## Step 2: Update project documentation
Only where something worth keeping has changed:
- Update `docs/STATUS.md` with the current state, following its own rules (concise, current state only, replace stale items).
- Save new research findings in `docs/research/`, design material in `docs/design/` and decisions in `docs/decisions/`, as described in the Project Documentation Workflow.
- If a spec looks like it needs a status change (for example Draft → Implemented), list the suggested change in the handoff and in your report. Do not edit it.
- Preserve research findings, design decisions, specifications and important technical details (measurements, pin assignments, parameter values, version numbers, file formats, gotchas).
- Do not copy the same information into several files. Put it in its proper home and link to it.
- If you are unsure whether existing documentation is still accurate, leave it and add a note to the handoff's open questions.

If the project has no documentation yet, create only `docs/SESSION_HANDOFF.md` (and `docs/STATUS.md` if it is missing) and list the other gaps in the handoff.

## Step 3: Create or update `docs/SESSION_HANDOFF.md`
`docs/STATUS.md` is the short, always-current summary. `docs/SESSION_HANDOFF.md` is the fuller briefing for resuming work: it covers the latest session and everything a new session needs to continue. Where details already live in another doc, summarize in one line and link to it.

Rewrite the file to describe the project as it is now. Do not keep appending old sessions; carry forward only what is still relevant. Use this structure, writing "None", "Not applicable" or "Unknown" rather than leaving a section out:

```markdown
# Session handoff

_Last updated: <date>_ · _Session focus: <one line>_

## Project overview and current state
<What the project is, in two or three sentences. Where it stands right now.>

## Completed milestones
- <Milestone> (<date if known>)

## Work performed this session
- <What changed and where>

## Important files and locations
| Path | Purpose |
|------|---------|
|      |         |

## Tests and verification
| Date | Command or check | Result |
|------|------------------|--------|
|      |                  |        |

Not verified this session:
- <Item, and why it was not checked>

## Known bugs, limitations and open questions
- 

## Decisions and rationale
- <Decision>: <why>. (<link to docs/decisions/ record if one exists>)

## Uncommitted changes and Git status
<Output summary of `git status`, or "No Git repository".>

## Suggested spec status changes (awaiting approval)
- <spec file>: <current> → <suggested>, because <reason>. Or "None".

## Next recommended milestone
<One milestone, with a short reason. Mark it as a recommendation, not an agreed plan.>

## How to resume
1. Read `AGENTS.md`, then this file and `docs/STATUS.md`.
2. <Setup needed: environment, tools, hardware, services to start.>
3. <Where to start: files to open, first concrete step.>
4. <Anything to check before continuing, for example unverified work above.>

## Confirmed facts vs assumptions
- Confirmed: <items checked against files, Git or commands in this session>
- Assumed: <items not checked, and why>
```

## Step 4: Verify
1. Re-read `docs/SESSION_HANDOFF.md` and `docs/STATUS.md` against the actual project files. Every path mentioned must exist; every test result must have come from a run you saw.
2. Check the two files agree with each other and with `docs/specs/` and `docs/decisions/`.
3. Make sure confirmed facts and assumptions are clearly separated.
4. Read the handoff as if you had never seen this conversation. If a step in **How to resume** depends on chat context, write that context into the file.

## Step 5: Prepare to close
Report to me, in this order:
1. Documentation files created or updated (with paths).
2. Uncommitted changes (from `git status`), or "No Git repository".
3. Whether a Git commit looks appropriate now, and a suggested commit message. Do not commit.
4. Any suggested spec status changes awaiting my approval.
5. Anything verified vs. anything left unverified.

Then stop. Do not start new work.

## Resuming in a new session
When a new session starts, or I say **"Resume from the session handoff"**:
1. Read `AGENTS.md`, then `docs/SESSION_HANDOFF.md` and `docs/STATUS.md` if they exist.
2. Quickly check the project files and Git status against the handoff and point out anything that no longer matches.
3. Summarize the current state and propose next steps.
4. Wait for my go-ahead before writing code or changing files.
