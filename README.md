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
- JGraphT for graph support
- JTS, H3, and S2 as pluggable spatial/topology extensions
- Jackson JSR-310 for time-aware JSON handling
- Lombok

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

The coverage gate excludes demo/experimental prototype surfaces that are not part of the current engine commitment, including demo classes, experimental routing search/replan packages, the old RRT planner prototype, and large Kalman/probabilistic estimator prototypes.

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
- Renamed the old polymorphic demo to `AirspaceMultiModelDemo`.
- Added pluggable spatial backends for JTS/H3/S2.
- Added JaCoCo coverage enforcement and raised enforced line coverage above 75%.
- Added regression fixtures under `src/test/resources` so the project is testable from the repository alone.

## Author

Tashdid Khan

