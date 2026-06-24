# AGENTS.md - Airspace

## Project

TODO: Describe what this project does.

## Architecture

TODO: Describe runtime, framework, database, deployment target, and major modules.

## Commands

Before changing code, inspect available commands:

```bash
find . -maxdepth 3 \( -name package.json -o -name pom.xml -o -name build.gradle -o -name pyproject.toml -o -name Cargo.toml -o -name wrangler.toml -o -name Makefile \)
```

Use discovered commands only. Do not invent commands.

## Standards

- Follow existing architecture.
- Keep changes small and reviewable.
- Add tests for changed behavior when practical.
- Include explicit error handling.
- Do not commit secrets or local environment files.
- Do not rewrite unrelated code.

## Restricted Paths

If this project touches client data, government systems, production infrastructure, auth, encryption, audit logging, or secrets:

- Do not read or modify secret files.
- Do not print environment variables.
- Do not access production systems.
- Do not run destructive database commands.
- Do not change authentication, encryption, audit logging, or access control without explicit approval.
- Prefer read-only analysis first.

## Definition Of Done

- Relevant tests pass or failures are explained.
- Changed files are summarized.
- Risks are documented.
- Follow-up tasks are listed.
