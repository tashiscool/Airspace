Read `AGENTS.md` and `.codex/commands.md`.

Goal:
Find and fix the smallest set of failing tests or build failures.

Process:
1. Run the smallest relevant validation command.
2. Identify the first real failure.
3. Fix only the cause of that failure.
4. Re-run validation.
5. Repeat until the relevant command passes or the remaining failure is outside this task.

Do not rewrite unrelated code.

Final response must include:
- commands run
- failures found
- changed files
- final result
- remaining risks
