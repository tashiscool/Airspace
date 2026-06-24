# Default Review Rubric

Use this rubric for code review and implementation self-checks.

## P0

- auth bypass
- secret exposure
- data loss
- broken deploy
- destructive migration without safeguards

## P1

- broken core workflow
- serious correctness bug
- unsafe input handling
- missing transactional or persistence safeguards
- unreliable external API integration

## P2

- missing tests for changed behavior
- brittle error handling
- unclear ownership boundaries
- performance issue in common path
- weak observability

## P3

- naming
- docs polish
- small simplifications
- minor UX polish
