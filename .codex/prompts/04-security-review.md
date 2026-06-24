Read `AGENTS.md`.

Perform a security review.

Do not edit files until after producing findings.

Look for:
- auth bypasses
- missing input validation
- unsafe deserialization
- SQL injection
- command injection
- path traversal
- SSRF
- secret leakage
- weak CORS
- unsafe logging
- missing webhook verification
- insecure defaults
- dependency risks

Output findings using:
- severity
- file
- line/function
- issue
- exploit scenario
- fix
- confidence

Then fix only P0/P1 issues with high confidence.
