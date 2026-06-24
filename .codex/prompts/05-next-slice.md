Read `AGENTS.md`, `PLANS.md`, `.codex/architecture.md`, `.codex/commands.md`, `.codex/backlog.md`, and `.codex/risks.md` if they exist.

If these files do not exist, create them by inspecting the repo. Do not make product changes yet.

Pick one task that is:
- valuable
- bounded
- testable
- unlikely to require product clarification
- safe to implement in a worktree

Before editing:
- state the selected task
- explain why it is the best next task
- list files likely to change
- list commands you will run
- list risks

Rules:
- no placeholder code
- no fake tests
- no broad rewrites
- no secret exposure
- no architecture drift
- no hiding failures
- prefer simple code
- add error handling
- add tests when practical

Validation:
Run the smallest relevant command first, then broader checks.

Final response:
1. changed files
2. behavior changed
3. tests added/updated
4. commands run
5. results
6. risks
7. next 3 tasks
