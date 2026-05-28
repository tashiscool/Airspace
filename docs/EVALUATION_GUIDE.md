# Evaluation Guide

This guide is for aviation safety reviewers, researchers, pilots, controllers, dispatchers, meteorologists, and public-interest technologists who want to evaluate Airspace without becoming a contractor or vendor.

## Quick Local Evaluation

Prerequisites:

- Java 17
- Maven
- Node.js
- npm

Run backend tests:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn test -q
```

Run frontend tests:

```bash
cd frontend
npm install
npm test -- --run
npm run build
```

Run the local product demo:

```bash
# Terminal 1
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn quarkus:dev \
  -Dquarkus.enforceBuildGoal=false \
  -Dquarkus.http.port=8090 \
  -Dquarkus.test.continuous-testing=disabled

# Terminal 2
cd frontend
npm run dev -- --host 127.0.0.1

# Browser
open http://127.0.0.1:5173
```

Default local users:

- `planner` / `planner`
- `supervisor` / `supervisor`
- `admin` / `admin`

Generate the screenshot walkthrough:

```bash
cd frontend
npx playwright install chromium
npm run screenshots
```

The screenshot script seeds demo data through the local APIs and writes images to `docs/screenshots/`.

## What To Review

### 1. Weather And PIREP Intake

Open the Weather page and verify:

- METAR/TAF/SIGMET/AIRMET/CWAP-style products are separated from PIREPs.
- Coordinate-bearing products create map features.
- Non-geometric METAR/TAF products remain table/guidance artifacts.
- Stale or low-confidence products are visually softened/dashed.
- Source refs stay clickable.

### 2. Affected Mission Detection

Open Mission Explorer and verify:

- active missions show weather verdicts,
- affected mission cards include action, confidence, source count, latency, and rationale,
- source chips route back to weather, PIREP, NOTAM, USNS, or CARF/ALTRV artifacts,
- attention filters surface blocked/severe/stale/rejected/unreviewed items.

### 3. Route Impact And Avoidance

Open Mission, Reservation, and Decision pages and verify:

- impacted segments and blocking constraints are visible,
- route alternatives show added distance, delay, fuel, cost, avoided hazards, and residual constraints,
- route-impact map layers remain distinct from reservations, NOTAMs, and conflicts,
- every AVOID/REROUTE/BLOCKED recommendation has source refs and rationale.

### 4. PIREP Relevance

Open Mission detail and verify:

- PIREPs can be scoped by route, altitude band, altitude tolerance, recency, and corridor buffer,
- stale/off-route/off-altitude reports are retained as context but marked lower relevance,
- urgent turbulence/icing reports are promoted to review/coordination guidance.

### 5. ATC/Weather Coordination

Use a Coordinate action and verify:

- the message draft includes mission, action, impact summary, and source refs,
- the draft does not send automatically,
- recipient/delivery behavior remains local/demo unless live adapters are explicitly added later.

### 6. Pilot Brief

Open `/missions/{missionId}/brief` and verify:

- the brief is read-only and printable,
- the current verdict, route impact, source artifacts, and what-changed items are visible,
- source refs are grouped by family,
- the trace summary does not claim official clearance authority.

### 7. Audit And Replay

Open Decision and History pages and verify:

- decision trace includes parse/classify/map/fuse/route-impact/action steps,
- audit and replay payloads are visible,
- source IDs are retained,
- replay verification returns diagnostics rather than silent acceptance.

## Review Output Template

When sending feedback, include:

- scenario or page reviewed,
- input fixture or screenshot,
- what the tool said,
- what a real operator would need instead,
- whether the issue affects safety, usability, traceability, or calibration,
- suggested fixture/test to add.

## Good First Scenario Contributions

- Convective SIGMET crossing a mission route with viable reroute.
- Severe icing PIREP near route altitude.
- Turbulence PIREP that is stale but still relevant.
- NOTAM restriction overlapping weather but not a CARF/ALTRV reservation.
- METAR/TAF low ceiling causing delay rather than reroute.
- CWAP-style/CWAF-like moving storm with later forecast blockage.
- Malformed USNS batch where one weather transaction is rejected.
