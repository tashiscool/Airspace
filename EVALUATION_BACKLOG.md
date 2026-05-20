# Airspace Evaluation Backlog

Every artifact below should be evaluated before calling the FAA/CARF/spatial-model solution comprehensive. For each item, record: what it contains, expected behavior, whether current code supports it, tests added, and any remaining gap.

## Priority 0 - Known Conflict/Reservation Regressions

- `~/Downloads/tworessies/` and `~/Downloads/tworessies.zip`
  - Evaluate as the smallest two-reservation reproduction.
  - Expected output: deterministic conflict/no-conflict result with time, lateral, longitudinal, and vertical reason.
  - Add regression tests against `ReservationConflictDetector`.
  - Status: initial parser regression added; named fixes, `ADMIS MITO`, and seconds-based admission are covered.

- `~/Downloads/runtheseandseewhyconflict/` and `~/Downloads/runtheseandseewhyconflict.zip`
  - Evaluate why the supplied reservations conflict.
  - Expected output: explain each conflict dimension and whether the old system or current model is correct.
  - Add parser fixtures and detector tests.
  - Status: initial head-on conflict regression added with conflict explanation assertions.

- `~/Downloads/conflicttoshort/`, `~/Downloads/conflicttoshort.zip`
  - Evaluate the “conflict too short” scenario and screenshot/listing.
  - Expected output: minimum duration/zero-duration conflict policy is explicit and tested.
  - Add threshold tests for instantaneous, tiny, and meaningful overlaps.
  - Status: listing parser regression added; detector now has configurable minimum conflict duration. The screenshot `90-91conflict.jpg` is inventoried as visual evidence with dimensions/hash and is duplicate-equivalent to the forwarded copy.

- `~/Downloads/fwdconflicttoshort/`, `~/Downloads/fwdconflicttoshort.zip`
  - Compare against `conflicttoshort`; identify duplicate vs changed evidence.
  - Expected output: one consolidated regression case unless files differ materially.
  - Status: byte-level duplicate evaluator added; duplicate conflict listing is marked reference-only. Visual hash coverage confirms the forwarded `90-91conflict.jpg` is identical to the original screenshot.

- `~/Downloads/messages.eml`, `~/Downloads/messages.zip`
  - Continue evaluating CARF timing examples, route timing, AVANA, TAS, and climb behavior.
  - Expected output: raw CARF A-G messages produce reservations equivalent to legacy examples.
  - Status: raw CARF timing parser and regression tests are in place. ZIP artifact extraction added; `messages.zip` now feeds the parser directly and reproduces the same-year WOODY/DEAL climb-profile conflict set with five AVANA-inclusive conflicts. The email wrapper contains the same timing attachments plus `tracks.JPG`, which remains visual reference evidence.

- `~/Downloads/ptr-02-A.txt`, `~/Downloads/ptr-02-B.txt`, `~/Downloads/PTR 002-R02-B-1.doc`
  - Evaluate PTR 002 conflict before/after evidence.
  - Related images: `ptr-02-original-conflict.png`, `ptr-02-now-conflict.png`.
  - Expected output: reproduce original issue and verify intended current behavior.
  - Status: raw CARF PTR 002 messages now parse with named fixes/airway fixes `ALB`, `BUF`, `YXU`, `ECK`, `PECOK`, and `GRB` resolved in the regression waypoint catalog. The Word artifact states the expected result is no conflict for the opposite-direction longitudinal-separation case; regression coverage now asserts the pair produces five reservations per side and no false conflict. The PNGs remain visual evidence for the old/incorrect display.

## Priority 1 - CARF Domain Rules and Legacy Spatial Model

- `~/Downloads/iRD - Protected Airspace Rules for CARF.doc`
  - Extract authoritative protected-airspace rules.
  - Expected output: map each rule to implemented behavior or a tracked gap.
  - Pay special attention to lateral, longitudinal, vertical, timing, AVANA, protected route width, maneuver areas, stationary reservations, orbit events, and turnaround handling.
  - Status: core separation principles extracted into `CarfProtectedAirspaceRules`; route pill/lateral segment distance and along-route longitudinal gap are now handled by deterministic local tangent-plane geometry instead of sample-grid approximation.

- `~/Downloads/CLEANCARF.backup`
  - Determine format and whether it contains legacy data/config.
  - Expected output: either importable fixtures/config or documented non-use.
  - Status: identified as a PostgreSQL custom database dump for `CarfdB2`. A lightweight dump analyzer now inventories schema/copy-table names without requiring a local Postgres restore and records whether `pg_restore` is available in the current environment. Relevant tables include `t_Separations`, `t_GroupSeparations`, `t_AirspaceMap`, `t_Navaids`, `t_Route`, `t_Reservation`, `t_CarfMessage`, `t_Notams`, and mission/message queues. Full row extraction is explicitly blocked here because `pg_restore` is not installed locally; it needs compatible Postgres restore tooling.

- `~/Downloads/AirspaceReservation.java`
  - Compare legacy reservation model fields to current `AirspaceReservation`.
  - Expected output: field coverage matrix and missing semantics.
  - Status: evaluated as design reference, not a copy candidate, because it depends on legacy `gov.faa` source/range types. The modern reservation model covers the durable semantics from the legacy class: start/end, AVANA expansion, longitudinal start/end expansion, vertical separation, deconfliction vertical range, source metadata, route endpoints, source ratios, shape intent, protected volume, and reservation type. Compatibility tests now pin the legacy temporal and vertical expansion behavior.

- `~/Downloads/TimingTriangleEvent.java`
  - Evaluate triangle/timing geometry rules.
  - Expected output: either implement route timing triangle support or document why current segment model covers it.
  - Status: modern event type and mapper support added. Timing triangles now validate exactly three area points and map to a single native `SpatialVolume` reservation, matching the legacy event's area-reservation behavior rather than synthetic route fragments.

- `~/Downloads/ManeuverAreaEvent.java`
  - Evaluate maneuver-area protected airspace behavior.
  - Expected output: spatial volume representation and conflict tests.
  - Status: modern event type and mapper support added. Maneuver areas now map to a single native protected-volume reservation; polygon areas use polygon volumes and point/short areas use a protected-radius circle fallback.

- `~/Downloads/OrbitEvent.java`
  - Evaluate orbit/holding/orbiting reservation behavior.
  - Expected output: support via holding pattern or a dedicated orbit reservation model.
  - Status: modern event type and mapper support added. Orbit events now validate the legacy 0-999 NM radius constraint and map to a single circular protected-volume reservation.

- `~/Downloads/StationaryReservationEvent.java`
  - Evaluate stationary reservation behavior.
  - Expected output: point/area volume support with temporal deconfliction tests.
  - Status: modern event type and stationary mapper support added. Stationary area reservations now produce native protected volumes from polygon or circular area inputs.

- `~/Downloads/PotentialConflict.java` and `~/Downloads/ActualConflict.java`
  - Compare legacy conflict state model to current `ReservationConflict`.
  - Expected output: ensure potential vs actual conflict semantics are represented.
  - Status: current conflicts now expose dimension explanation, duration, required separation, raw route ratios, and factored AVANA/longitudinal route ratios. Legacy `ActualConflict` ratio/date math was modernized into `ConflictRatioCalculator`; effective conflict end now includes AVANA plus longitudinal padding to match legacy `endDateWithAvanaAndLong` semantics.

- `~/Downloads/RouteSegmentSource.java`
  - Evaluate route source metadata: fixes, ratios, points, route segment distance, separation values.
  - Expected output: preserve enough source data for debug/display and test diagnostics.
  - Status: reservation metadata expanded with source fixes, ratios, route width, shape intent, and source text.

- `~/Downloads/DeconflictionContext.java`, `DeconflictionEntityType.java`, `DeconflictionTester.java`, `DeconflictionUtils.java`, `EntityFactory.java`
  - Evaluate old deconfliction engine API and utility behavior.
  - Expected output: identify reusable algorithms, missing scenario types, and expected fixture runner behavior.
  - Status: `EntityFactory` strip/capsule route geometry has been modernized into native geodetic `SpatialPolygon`/`SpatialVolume` construction. Raw CARF route reservations and modern route-segment events now carry capsule protected volumes. Rendering-independent polygon padding from the legacy `processSeparationCriteria` path is represented by geodetic protected-area polygon expansion for timing-triangle, maneuver-area, and stationary polygon events when route width/protected radius is provided. `DeconflictionContext` route-leg splitting behavior is now represented by injectable `CarfRouteSeparationResolver`/`CarfSeparationStandard` support: when standards differ across a leg, the parser splits the reservation at a resolved transition ratio and preserves per-segment lateral, vertical, longitudinal, route-width, time, source-ratio, and protected-volume metadata. `DeconflictionTester` conflict diagnostics are now represented on `ReservationConflict` with conflict-window route points, distance-at-start, distance-at-end, and angle-between-routes fields. Luciad entity creation and drawing styles remain reference-only unless a rendering-independent rule requires them.

## Priority 2 - DOM/NOTAM Parser Completeness

- `~/Downloads/Dom1.ycc`, `~/Downloads/Dom1.zip`, `~/Downloads/Dom1-1.zip`, `~/Downloads/Dom1/`, `~/Downloads/Dom1 2/`
  - Evaluate all DOM1 grammar variants and generated Java files.
  - Expected output: domestic NOTAM parser supports all token/production behavior that matters in current scope.
  - Status: DOM1 structural tests and structured parse result are in place.

- `~/Downloads/Dom2.ycc`, `~/Downloads/newDom2.ycc`, `~/Downloads/domparse2.ycc`, `~/Downloads/Dom2.zip`, `~/Downloads/Dom2/`
  - Evaluate DOM2 contraction/q-code grammar.
  - Expected output: clarify whether current solution needs full contraction/q-code reduction or only structural parse support.
  - Status: contraction dictionary scaffold added for DOM2 keyword clarification terms, and `DomesticNotamContractionDictionary.fromLegacyYcc` can now load the full uncommented contraction vocabulary from `Dom2.ycc`/`newDom2.ycc` without copying generated parser tables. A modern q-code classifier exposes selected `q23`/`q45` reducer outputs from concrete DOM2 cases: NAV VOR/VORTAC/DME/TACAN/NDB, guard lights, PJE airspace, ARFF, AWOS, ATIS, RVR, and PCL lighting. Full generated-parser reducer parity remains pending only if exact legacy q-code output is required for every DOM2 production.

- `~/Downloads/Dom2Diff.htm`, `~/Downloads/OriginalVNewDom2.htm`
  - Extract parser diffs and confirm all meaningful changes are reflected.
  - Expected output: checklist of added/changed tokens, productions, and reduce behavior.
  - Status: visible diff additions such as `UNABL`, `TAXILANE(S)`, `TAXIS`, `APCH`, `UNAVBL`, and `SFC` are covered by tests.

- `~/Downloads/fwdfwdkeywordsthatneedclarification/`, `~/Downloads/fwdfwdkeywordsthatneedclarification.zip`
  - Evaluate keyword clarification cases.
  - Expected output: parser accepts/rejects each case with documented reason.
  - Status: legacy BIFF8 spreadsheet text extraction now scans embedded UTF-16LE strings in addition to ASCII byte runs. Representative executable NOTAM rows from all 14 keyword-clarification spreadsheets are covered by parser/dictionary tests: APCH, apron taxilane, disabled aircraft, dismantled aircraft, FT-width text, guard lights, moored ship, RAMP PAEW, SFC, ski strip, snow/ice, turnarounds, UNAVBL, and UNMON. Full row-by-row q-code/reduce classification remains pending if exact DOM2 reducer parity becomes required.

- `~/Downloads/PTR 139-C-2.doc`, `~/Downloads/PTR 148-B-2.doc`, `~/Downloads/JR 16 August 2010.doc`
  - Extract parser requirements, bug reports, and examples.
  - Expected output: tests for every concrete NOTAM example and rule.
  - Status: PTR 139 BEGIN AIRFL and PTR 148 radial/DME route examples are covered; JR 30-minute passage-window rule is covered in the rules catalog.

- `~/Downloads/Visual Parser Files.eml`, `~/Downloads/Big Zip file for parser tester...and that's what she said.eml`, `~/Downloads/Bundle.eml`
  - Finish mining old parser tester setup, workspace, generated classes, and test data.
  - Expected output: no useful parser behavior remains only in attachments.
  - Status: email attachment inventory added. `Visual Parser Files.eml` contains `Dom1.zip` with the Visual Parser++ workspace/generated files (`Dom1.ycc`, generated Java tables/classes, `TESTfILE.txt`) and a `VisualParser++.rar` attachment that is actually ZIP-formatted. The “Big Zip” email contains only `com.ibm.ws.admin.client_6.1.0.zip`, which contains the WebSphere admin client JAR and is vendor/runtime baggage, not parser behavior. `Bundle.eml` contains `davorJunk.zip` with `Dom1.zip`, `VisualParser++.rar`, setup instructions, `DaveWorkspace.zip`, and `OriginialWorkspace.zip`; tests now confirm `DaveWorkspace` contains `DomNotamParserTester` timestamped text fixtures and compiled parser classes, while `OriginialWorkspace` contains legacy workspace source/grammar files such as `domparse2.ycc` and app-client Java sources. The timestamped domestic parser tester fixtures now contribute tests for valid `!DCA LDN NAV VOR OTS WEF 0708051600-0708052359` parsing and malformed `WEF 07A08051600` rejection. Remaining gap: repeated ICAO/FDC-style tester outputs are legacy runtime-crash evidence, not yet structured parser cases.

## Priority 3 - Navaids, Reference Data, and Account Tables

- `~/Downloads/navaids/`, `~/Downloads/navaids.zip`, `~/Downloads/ace3411.txt`
  - Evaluate navaid/fix coordinate data.
  - Expected output: determine whether route parsing must resolve named navaids/fixes, not only coordinates.
  - Status: waypoint resolver API and default regression catalog added; parser diagnostics now expose `resolvedWaypointNames` and `unresolvedWaypointNames` so synthetic fallback coordinates are visible in tests and callers. `CompositeCarfWaypointResolver` supports KVM/NASR/DAFIF/database integrations ahead of the built-in fixture catalog, and `WaypointFileResolver` loads simple CSV/whitespace exports with identifier, latitude, longitude, and optional altitude. Parser-level tests prove an external waypoint export can resolve CARF route fixes before fallback catalog lookup. Legacy autodraw/DAFIF duplicate-navaid rules are ported into a modern prioritized resolver: explicit preferred entries win, then US/FAA-region country codes (`K`, `PA`, `PH`, `TI`, `TJ`), then first foreign fallback. The local `navaids/` artifacts are CARF route-control examples rather than standalone navaid tables; `ace34l1`, `ACE2`, and `ACE70L2` now parse with named route points resolved and no hidden fallback fixes. Full live KVM/database client wiring remains an integration layer rather than hard-coded project data.

- `~/Downloads/GLOBALACCOUNTS.xls`, `~/Downloads/GLOBALACCOUNTSreaL.xls`
  - Evaluate global-account keyword insertion behavior from legacy DOM parser.
  - Expected output: support or document account-based defaults such as GPS -> NAV and other globals -> AIRSPACE.
  - Status: dependency-free legacy spreadsheet string extraction and global-account resolver added.

- `~/Downloads/CARF Fax Number.xls`
  - Evaluate whether operational contact data is relevant.
  - Expected output: likely metadata only, but confirm no parser/config values are hidden there.
  - Status: classified with a legacy spreadsheet analyzer as BIFF8/compound-document contact metadata. Extractable strings include facilities/contacts such as `ZNY TMU`, `ZAU TMU`, `ZKC TMU`, `ZOATMU`, `VACAPES`, ranges, and a phone-like numeric token, but no executable CARF/NOTAM rows. Marked reference-only for framework behavior.

- `~/Downloads/carftesting.xls`
  - Evaluate test matrix or expected outcomes.
  - Expected output: convert relevant rows into automated tests.
  - Status: classified with a legacy spreadsheet analyzer as BIFF8/compound-document workbook with sheet names `Sheet1`, `Sheet2`, and `Sheet3`, but no extractable executable CARF/NOTAM/conflict rows using available local tooling. Marked reference-only unless a BIFF8-compatible parser/converter becomes available.

- `~/Downloads/AEROBAT.xls`, `BERMS.xls`, `FIRs.xls`, `PILE.xls`, `REMAINING.xls`, `TURNAROUND.xls`
  - Evaluate CARF category/config spreadsheets.
  - Expected output: import rules, fixture data, or explicit exclusion with rationale.
  - Status: dependency-free legacy spreadsheet string extraction added; NOTAM-like rows are extractable and covered for AEROBAT, BERMS, PILE, REMAINING, and TURNAROUND. Domestic AIRSPACE NMR/radial-DME rows from AEROBAT now parse into native spatial volumes. Domestic surface-condition rows now parse structured berm, snow-pile, turnaround, snow, ice, slush, frost, water, and deiced-liquid hazards with surface identifiers and measured berm/pile heights where present. REMAINING rows now parse runway distance-remaining sign/marker status, distances, and statuses such as OTS, UNLGTD, OBSC, MISSING, REMOVED, NONSTD, and NON-CONFORMING. Coordinate-bearing FDC airborne-laser records now parse into native spatial restrictions.
  - FIRs appears to be account/location/FIR reference data rather than executable NOTAM rows. A lightweight `FirReferenceCatalog` now extracts legacy identifiers and descriptions, with coverage for FAA centers/FIR-style identifiers such as `ZOA`, `ZAN`, `ZBW`, `CZVR`, and airport identifiers such as `KJFK`. Full BIFF row-level relationships remain pending if account-to-FIR mapping becomes an operational resolver input.

## Priority 4 - Display, Autodraw, and Visual Evidence

- `~/Downloads/autodrawFiles/`, `~/Downloads/autodrawFiles.zip`
  - Evaluate old CARF drawing/display code.
  - Expected output: identify shape-generation requirements for deconfliction, normal display, and conflict display.
  - Status: evaluated as reference-only for direct copy because the files depend heavily on Luciad, legacy `gov.faa.*` packages, Swing UI, and JAXB config classes. Added Java artifact dependency analyzer coverage. Reusable duplicate-navaid selection behavior from `CarfDafifManager` was modernized into `PrioritizedCarfWaypointResolver`; display-specific rendering remains pending as rendering-independent geometry requirements.

- `~/Downloads/conflict display.eml`
  - Reattempt extraction of encrypted/unsupported `display.rar` if a compatible RAR tool/password is available.
  - Expected output: determine whether there are display fixtures or geometry rendering rules inside.
  - Status: blocked with test coverage. The email attachment extractor finds `display.rar`; RAR header analysis confirms encrypted headers. Local tools include `bsdtar`, but it reports RAR encryption support unavailable. Needs password and compatible RAR tooling before content can be evaluated.

- `~/Downloads/carf-checkin.png`
  - Evaluate screenshot for UI/data expectations.
  - Status: inventoried as non-executable visual evidence (674x436 PNG) with test-backed dimensions/hash. No parser or geometry rule is uniquely derivable from the screenshot alone.

- `~/Downloads/ptr-02-original-conflict.png`, `~/Downloads/ptr-02-now-conflict.png`
  - Use as visual acceptance evidence for PTR 002.
  - Status: inventoried as reference-only visual evidence with test-backed dimensions/hash. The executable PTR 002 behavior is covered by raw-message parsing and no-false-conflict assertions; the PNGs document the old vs corrected display symptom.

- `~/Downloads/attachments/`, `~/Downloads/attachments.zip`
  - Evaluate all attached documents/images, especially `PTR 045-B-1.doc`.
  - Expected output: convert scenarios into tests or mark as non-executable reference.
  - Status: `HEADON ONE.txt`, `HEADON TWO.txt`, and `PTR 045-B-1.doc` are covered. PTR 045 documents a false conflict after a climb; the attached no-CLMB pair now parses shorthand longitudes such as `9503W`/`9450W`, preserves coordinate route fixes, and asserts no conflict because vertical separation is met. The current `attachments/` folder contains only those three files, so no remaining attachment images/docs are present locally to classify.

## Priority 5 - Source Archives and Legacy Workspaces

- `~/Downloads/src/`, `~/Downloads/src 2/`, `~/Downloads/src.zip`, `~/Downloads/src (2).zip`
  - Compare legacy source snapshots with current project.
  - Expected output: identify algorithms/classes still missing from current implementation.
  - Status: ZIP comparison harness added. `src (2).zip` is confirmed as the newer parser reference; it adds `FileCreator.java`/`Main.java` and changes generated/legacy parser files including `DomNotam.java`, `Dom1YaccClass.java`, and `Dom1LexClass.java`. Direct copy is still avoided because the archive is dominated by generated parser tables, EJB/DAO code, and legacy package dependencies.

- `~/Downloads/mrkoci/`, `~/Downloads/mrkoci.zip`
  - Evaluate additional old code or documents.
  - Expected output: extract scenario coverage or reusable code.
  - Status: ZIP contains two CARF lateral/radial-DME regression messages (`JR-TEST-LAT1A.txt`, `JR-TEST-LAT1B.txt`) plus `JRLAT-TEST.png`. The message pair now parses into five reservations per side, including radial/DME fixes around GSH and GUS, and produces 18 explainable conflicts with protected volumes and start/end/angle diagnostics after replacing sampled route distance with deterministic segment geometry. The PNG is inventoried as reference-only visual evidence (1039x683) with test-backed dimensions/hash.

- `~/Downloads/reenjoiitaly___buttheresmorecarfwork.zip`
  - Evaluate remaining CARF work package.
  - Expected output: scenario list and missing feature list.
  - Status: classified as the packaged legacy deconfliction/event source bundle containing `ActualConflict`, `PotentialConflict`, `RouteSegmentSource`, `DeconflictionContext`, `DeconflictionUtils`, `DeconflictionTester`, `EntityFactory`, and the event classes. It is now covered as source-material evidence; reusable behavior already ported includes ratio/date math, AVANA/longitudinal windows, route-segment metadata, conflict diagnostics, strip/capsule geometry, event mapping, and airspace-standard leg splitting. Direct copy remains inappropriate because the bundle depends on Luciad/JUNG/legacy `gov.faa` APIs.
  - File-by-file coverage matrix:
    - `ActualConflict.java`: covered by `ConflictRatioCalculator`, AVANA/longitudinal factored windows, conflict start/end ratios, conflict-window diagnostics, deterministic segment intersection, and along-route longitudinal gap calculation. Remaining gap: exact legacy Luciad recursive boundary search is not copied; current geometry is rendering-independent and test-backed.
    - `PotentialConflict.java`: covered by effective start/end overlap, AVANA/longitudinal expansion, and vertical deconfliction-range overlap semantics.
    - `RouteSegmentSource.java`: covered by route source fixes, source ratios, route segment distance, lateral separation, route width, route endpoints, protected volumes, and generated sub-window conflict points. Remaining gap: legacy display shape list is represented as geometry/shape intent, not Luciad shapes.
    - `AirspaceReservation.java`: covered by modern reservation temporal/vertical/source metadata compatibility tests.
    - `DeconflictionContext.java`: covered by route-leg generation, event mapping, flight-level merge behavior, and airspace-standard transition splitting. Remaining gap: complete route-graph traversal semantics are represented by parser/event tests, not a JUNG graph clone.
    - `DeconflictionEntityType.java`: covered by normal vs conflictor expansion semantics through protected geometry and conflict diagnostics; no legacy enum package copied.
    - `DeconflictionTester.java`: covered by parser fixture tests, conflict diagnostics, distance-at-start/end, angle-between-routes, and archive-driven regressions.
    - `DeconflictionUtils.java`: covered for default separations, fix/radial-DME coordinate resolution, vertical ranges, area/orbit/stationary/timing event behavior, and navaid resolver hooks. Remaining gap: full Postgres/DAFIF-backed airspace lookup is still pending; `CLEANCARF.backup` has been inventoried but not restored.
    - `EntityFactory.java`: covered for strip, capsule, circle-derived volumes, route protected volumes, and rendering-independent polygon padding/expansion for protected area events. Remaining gap: exact Luciad edge-intersection offset behavior is approximated by centroid-based geodetic expansion unless a fixture proves that exact boundary construction is required.
    - `ManeuverAreaEvent.java`: covered by maneuver-area event mapping to native `SpatialVolume`.
    - `OrbitEvent.java`: covered by orbit radius validation and circular protected-volume mapping. Remaining gap: nested transition-event subclasses are not fully modeled beyond merged flight-level behavior.
    - `StationaryReservationEvent.java`: covered by stationary point/polygon area protected-volume mapping.
    - `TimingTriangleEvent.java`: covered by exactly-three-point validation and timing-triangle protected-volume mapping.

- `~/Downloads/texts/`, `~/Downloads/texts.zip`
  - Evaluate plain-text samples.
  - Expected output: classify into NOTAM, CARF, conflict listing, or reference text; add parser fixtures.
  - Status: timing text fixtures are covered. The extracted folder has the DEAL climb-profile fixture in 2015, so it is asserted as non-conflicting against the 2010 WOODY baseline while preserving climb altitude-profile semantics.

## Completion Criteria

- Every listed artifact has an evaluation note.
- Every concrete example has an automated test or a documented reason it cannot be executed.
- CARF conflict decisions explain time, lateral, longitudinal, and vertical dimensions.
- Domestic NOTAM parsing covers DOM1/DOM2 examples, expanded keywords, Canadian crossovers, comments, cancellations, WEF/TIL, PERM, EST, 10/12-digit times, punctuation, and malformed input.
- Spatial model supports line, arc, route segment, stationary, orbit/holding, maneuver area, timing triangle, protected airspace buffers, and display-shape generation where required.
- Full `mvn test` passes after each evaluation batch.
