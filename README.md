# Airspace

Airspace is a Java 17 framework for modeling airspace reservations, legacy FAA-style messaging, NOTAM constraints, aviation weather hazards, PIREPs, route impact, and explainable operational decisions.

The project started as a general airspace-modeling sandbox, but the current codebase is now centered on an engine that can turn CARF/ALTRV reservations, USNS traffic, NOTAMs, weather products, and aircraft reports into structured constraints, conflict checks, route blockage predictions, and auditable recommendations.

## What Is Implemented

### CARF / ALTRV / Reservation Engine

- CARF route-message parsing and reservation construction.
- Modern ALTRV parser package under `org.tash.extensions.carf.altrv`.
- A-G section/domain models, route events, route graph building, route graph validation, diagnostics, and spatial mapping.
- Reservation conflict detection with explicit lateral, longitudinal, vertical, and time explanations.
- Regression coverage for the 2010 legacy CARF examples that motivated the project, including:
  - lateral-separation-safe parallel routes,
  - head-on examples,
  - too-short/zero-duration conflicts,
  - navaid/fix-backed route examples.
- CARF reference-data abstractions and schema catalog under `org.tash.extensions.carf.refdata`.

Key entry points:

```java
CarfAnalysisService service = new CarfAnalysisService();
CarfAnalysisResult result = service.parseValidateMap(rawCarfOrAltrvText);
```

### USNS / Messaging / NOTAM Compatibility

- USNS envelope parsing for MRS/NADIN/WMSCR-style traffic.
- Transaction splitting and family classification for DOM, FDC, ICAO NOTAM, Canadian domestic, SNOWTAM, BIRDTAM, ASHTAM, GENOT, service requests, CARF-like bodies, and weather/advisory traffic.
- Batch-oriented parse diagnostics and routing outcomes.
- Domestic NOTAM parser support for DOM1/DOM2-style grammar behavior, cancellations, edits, comments, WEF/TIL, PERM/EST, 10/12-digit date forms, keyword defaults, and q-code/contraction classification.
- NOTAM access-policy abstractions and in-memory reference data.

Key entry point:

```java
UsnsIngestService ingest = new UsnsIngestService();
UsnsIngestResult result = ingest.parse(rawUsnsMessage);
```

### Weather Decision Engine

- Structured weather product model for convection, turbulence, icing, ceiling, visibility, SIGMET, AIRMET, METAR, TAF, NEXRAD/CWAP/CWAF-style advisories, PIREP-derived hazards, and generic forecast hazards.
- Product-specific decoders for pragmatic METAR, TAF, SIGMET, AIRMET, CWAP/CWAF, and PIREP parsing.
- Forecast slicing for TAF/CWAP-style products.
- Route weather decision support with actions such as clear, monitor, caution, delay, altitude change, reroute, avoid, and blocked.
- Route blockage prediction with severity, echo tops, growth/decay, storm phase, lead-time confidence, stale-product diagnostics, ensemble uncertainty, and capacity-impact seams.
- PIREP ingestion, duplicate detection, quality diagnostics, automated draft capture, and dissemination status modeling.
- ATC/weather coordination models for review items, operational constraints, controller handoff notes, and meteorologist review priority.

Key entry point:

```java
WeatherDecisionSupportService weather = new WeatherDecisionSupportService();
RouteWeatherAdvisory advisory = weather.adviseRoute(request);
```

### Unified Operational Decision Engine

- Top-level fusion facade under `org.tash.extensions.engine`.
- Fuses raw USNS messages, raw CARF/ALTRV text, structured weather products, PIREPs, NOTAM restrictions, route candidates, reference data, and decision time.
- Produces reservations, conflicts, weather products, PIREP results, route blockage predictions, coordination advisories, recommended actions, and a structured decision trace.
- Includes deterministic audit/replay support:
  - canonical JSON,
  - HMAC signing,
  - request/result hashes,
  - engine/rule-catalog versions,
  - replay verification.

Key entry point:

```java
OperationalDecisionEngine engine = new OperationalDecisionEngine();
OperationalDecisionResult result = engine.evaluate(request);
```

### Operations Product Layer

- Quarkus product API resources under `org.tash.extensions.product.api` for auth, missions, messages, feed ingest, decisions, reference data, config, history, and metrics.
- Product DTOs keep HTTP payloads separate from engine models and persistence entities.
- Baseline PostgreSQL/Flyway schema under `src/main/resources/db/migration`.
- JPA entity/repository seams under `org.tash.extensions.product.persistence` for missions, reservations, messages, NOTAMs, APREQs, approvals, weather products, PIREPs, operational decisions, users, reference points, and parsed ALTRV route graph targets.
- Quarkus ORM/Flyway/JDBC wiring is active: dev/test use H2 in PostgreSQL mode, while prod points at PostgreSQL through `AIRSPACE_JDBC_URL`, `AIRSPACE_DB_USERNAME`, and `AIRSPACE_DB_PASSWORD`.
- `ProductPersistenceService` provides transactional JPA persistence for product aggregates; the product API can run in in-memory mode or persisted mode through `airspace.product.persistence.enabled`.
- Persisted product aggregates currently include missions, mission locks, reservation workflow summaries, messages, weather products, and operational decisions/audit payloads.
- Persisted decisions can be reloaded into typed decision/audit/replay objects for restart-safe replay checks and GeoJSON feature generation. If a historical payload cannot be fully rehydrated, the API falls back to stored JSON hash/signature verification and returns explicit diagnostics.
- Product workflow APIs include distinct reservation parse/deconflict and force-parse/force-deconflict actions, message reply/forward actions, feed transaction inspection, reservation supplements, decision replay verification, and decision feature export.
- Reservation supplement lifecycle transitions are exposed for NOTAM/APREQ/approval-style records so operator actions can move draft/open records through submitted, approved, rejected, or cancelled states.
- Product search surfaces cover missions, messages, feed artifacts, decisions, and history, with a `ProductSearchService` facade for typed search entry points.
- The second product migration captures the useful legacy `MissionModelMapping.hbm.xml` shape without restoring legacy Hibernate/XML runtime: ALTRV messages, route groups, routes, route events, fix-times, areas, exits/destinations, notes/images, message recipients, airspace info, separation groups, aircraft validation, and preferred navaids.
- Local adapter seams are present for USNS/NADIN/WMSCR traffic, weather traffic, KVM-style bridge input, and reference-data sync. The checked-in implementations are file/in-memory replay adapters only; they are shaped for future live adapters without requiring credentials or network services.
- Reference-data import preview/apply endpoints accept local CSV-style navaid/fix/aerodrome/global-account records and preserve source/version metadata in reference-point records.
- Local RBAC auth is available for product development with default users:
  - `planner` / `planner`
  - `supervisor` / `supervisor`
  - `admin` / `admin`
- When `airspace.product.auth.required=true`, product APIs enforce role-aware workflow access: planners can create/edit/submit, supervisors can approve/reject, and admins can manage reference/config surfaces.

The first product API path is intentionally local/test-friendly: the engine remains usable without a live database, while the production Postgres schema and JPA model are present for deployment hardening.

### React Operations Console

- A React + TypeScript + Vite frontend lives in `frontend/`.
- It provides the initial operations shell for login, a four-pane CARF-style Mission Explorer, mission/reservation workspace, reservation save/lock/parse/submit/approve/reject/cancel/complete actions, deconfliction review, messaging, decision review, search, history, and config.
- The map stack is OpenLayers and consumes the backend’s GeoJSON-compatible feature collections.

Frontend commands:

```bash
cd frontend
npm install
npm test -- --run
npm run build
```

Screenshot walkthrough commands:

```bash
# Terminal 1: local API with H2/PostgreSQL-mode dev storage
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn quarkus:dev -Dquarkus.enforceBuildGoal=false -Dquarkus.http.port=8090 -Dquarkus.test.continuous-testing=disabled

# Terminal 2: browser console
cd frontend
npm run dev -- --host 127.0.0.1

# Terminal 3: seed demo data through the real APIs and capture product screenshots
cd frontend
npx playwright install chromium
npm run screenshots
```

The screenshot script uses the real local product APIs to create a mission, reservation, NOTAM/APREQ/approval supplements, USNS/feed traffic, weather/PIREP messages, reference points, and a replayable operational decision. It then drives the React workbench with Playwright and writes PNGs to `docs/screenshots/`.

### Product Walkthrough

The current operator-facing product path is working locally as an integrated prototype. These screenshots are generated from the checked-in app, not mockups.

![Login](/docs/screenshots/01-login.png)

| Mission Explorer | Mission Workspace |
| --- | --- |
| ![Mission Explorer](/docs/screenshots/02-mission-explorer.png) | ![Mission Workspace](/docs/screenshots/03-mission-workspace.png) |

| Reservation Sections | Reservation Supplements |
| --- | --- |
| ![Reservation Sections](/docs/screenshots/04-reservation-sections.png) | ![Reservation Supplements](/docs/screenshots/05-reservation-supplements.png) |

| Deconfliction Review | Messaging |
| --- | --- |
| ![Deconfliction Review](/docs/screenshots/06-deconfliction-review.png) | ![Messaging](/docs/screenshots/07-messaging.png) |

| USNS Feed | Decision Summary |
| --- | --- |
| ![USNS Feed](/docs/screenshots/08-usns-feed.png) | ![Decision Summary](/docs/screenshots/09-decision-summary.png) |

| Decision Trace | Decision Map |
| --- | --- |
| ![Decision Trace](/docs/screenshots/10-decision-trace.png) | ![Decision Map](/docs/screenshots/11-decision-map.png) |

| NOTAM Constraints | Weather And PIREPs |
| --- | --- |
| ![NOTAM Constraints](/docs/screenshots/12-notam-constraints.png) | ![Weather And PIREPs](/docs/screenshots/13-weather-pirep.png) |

| Config / Reference Data | Search |
| --- | --- |
| ![Config Reference](/docs/screenshots/14-config-reference.png) | ![Search](/docs/screenshots/15-search.png) |

![History And Audit](/docs/screenshots/16-history-audit.png)

Local product dev:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn quarkus:dev -Dquarkus.enforceBuildGoal=false -Dquarkus.http.port=8090 -Dquarkus.test.continuous-testing=disabled
cd frontend
npm run dev -- --host 127.0.0.1
```

The Vite dev proxy defaults to `http://localhost:8090` and can be overridden with `VITE_API_PROXY_TARGET`.

### Product Replay Corpus

A self-contained product replay corpus lives under:

```text
src/test/resources/scenarios/product-replay/
```

It includes mixed USNS/weather traffic, a CARF/ALTRV reservation, and an expected summary. `ProductReplayCorpusTest` verifies the local feed adapter, transaction inspection, operational decision evaluation, audit/replay payloads, and metrics without external files or live FAA/NWS/SWIM feeds.

### Spatial / Geometry / Visualization

- Core spatial model: points, lines, polygons, circles, arcs, volumes, trajectories, time intervals, wind fields, and 3D volume primitives.
- Engine geometry service for overlap, route-corridor, distance, segment-impact, and bbox prefilter behavior.
- Pluggable topology extensions:
  - native geometry fallback,
  - JTS for precise planar topology,
  - H3 for discrete cell indexing,
  - S2 for discrete cell indexing.
- Constraint spatial index with time buckets, altitude buckets, bbox filtering, product-validity filtering, and optional discrete topology cells.
- Rendering-neutral visualization model and GeoJSON exporter. Output is generic GeoJSON-compatible feature collections that can be consumed by Leaflet, OpenLayers, Mapbox, or another browser map layer.

Key entry points:

```java
OperationalGeometryService geometry =
    new PluggableOperationalGeometryService();

AirspaceVisualizationService visualization =
    new AirspaceVisualizationService();
```

### Reservation Workflow

- Engine/domain workflow support for reservation drafts, validation, submission, approval, rejection, cancellation, completion, locks, stale locks, conflict review, and audit events.
- In-memory and JSON-file repository implementations.
- Quarkus REST resources exist for local testing and integration, but the core framework does not require HTTP or persistence to use the engine.

## Current Technology Stack

- Java 17
- Maven
- JUnit 5
- JaCoCo coverage gate
- Quarkus REST/Jackson for optional local HTTP resources
- PostgreSQL-shaped Flyway migrations and JPA entity/repository seams
- JGraphT for graph support
- JTS, H3, and S2 as pluggable spatial/topology extensions
- Jackson JSR-310 for time-aware JSON handling
- Lombok
- React, TypeScript, Vite, TanStack Query/Table, React Hook Form, Zustand, and OpenLayers for the browser console

The README previously mentioned Spring Boot, React, Kafka, RabbitMQ, Oracle, DynamoDB, MongoDB, Docker, and Kubernetes as if they were implemented. They are not part of the current checked-in runtime. Live FAA/NWS/SWIM feeds, certified cockpit UI behavior, production databases, and deployment infrastructure are future adapter/deployment concerns.

## Test Coverage And Verification

Run the full test and coverage gate:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn verify -q
```

Current status from the latest verification run:

- Tests: 195
- Failures: 0
- Errors: 0
- Enforced JaCoCo line coverage: 77.5%
- Active framework coverage:
  - `org.tash.extensions.engine`: 85.3%
  - `org.tash.extensions.weather`: 91.6%
  - `org.tash.extensions.messaging`: 85.7%
  - `org.tash.extensions.carf`: 88.1%
  - `org.tash.extensions.notam`: 81.9%

Coverage report:

```text
target/site/jacoco/index.html
```

The coverage gate excludes walkthrough and experimental prototype surfaces that are not part of the current engine commitment, including executable walkthrough classes, experimental routing search/replan packages, the old RRT planner prototype, and large Kalman/probabilistic estimator prototypes.

## Test Fixtures

The project no longer depends on files in `~/Downloads`. Legacy evidence that remains relevant has been copied into test resources:

```text
src/test/resources/legacy/carf/
src/test/resources/scenarios/weather-engine/
```

Examples include CARF reservation text, head-on and lateral-separation regressions, navaid/fix fixtures, PTR-style examples, METAR/SPECI corpora, TAF corpora, SIGMET/AIRMET corpora, CWAP/CWAF examples, PIREP examples, and mixed USNS/CARF/NOTAM/weather scenarios.

## Useful Commands

Run all tests without coverage report generation:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn test -q
```

Run verification with coverage:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn verify -q
```

Build the browser console:

```bash
cd frontend && npm run build
```

Run one test class:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn -q -Dtest=OperationalDecisionEngineTest test
```

## Project Boundaries

Implemented as engine/framework code:

- CARF/ALTRV parsing, mapping, and conflict checks.
- USNS-style message parsing and transaction routing.
- Domestic NOTAM parsing and structured NOTAM restrictions.
- Aviation weather product modeling and pragmatic decoding.
- PIREP validation and quality diagnostics.
- Route blockage and operational decision support.
- Structured decision traces, rule catalog references, audit/replay envelopes, and deterministic local signing.
- Visualization-neutral GeoJSON feature generation.
- Restart-safe product decision replay verification and persisted decision feature export.
- Local product workflow APIs for reservation parse/deconflict, message reply/forward, feed transaction inspection, and RBAC-gated operator actions.

Not implemented as production integrations:

- Live FAA/NWS/SWIM data feed adapters.
- Certified FAA/NWS decoders.
- Certified cockpit UI.
- Live database/KVM connectivity.
- Operational alert dissemination.
- Production deployment, monitoring, or infrastructure automation.

## Repository Notes From Recent Work

The last 24 hours of commits moved the repo substantially:

- Completed the modern CARF/ALTRV, USNS/NOTAM, weather, PIREP, workflow, visualization, and operational decision engine layers.
- Removed final-product legacy evaluation utilities that referenced external artifact locations.
- Renamed the old polymorphic model runner to `AirspaceModelWalkthrough`.
- Added pluggable spatial backends for JTS/H3/S2.
- Added JaCoCo coverage enforcement and raised enforced line coverage above 75%.
- Added regression fixtures under `src/test/resources` so the project is testable from the repository alone.

## Author

Tashdid Khan
