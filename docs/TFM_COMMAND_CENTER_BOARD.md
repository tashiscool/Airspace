# Traffic Flow Management Command Center Board

Airspace now includes a local/replay-backed TFM command-center board for the operational-management lane: airport demand, sector load, active constraints, proposed Traffic Management Initiatives, route alternatives, impact totals, and human-review assumptions.

This is a production-grade prototype surface, not a live FAA FMDS/SMART/TFMS replacement and not an operational authority.

## Research Anchors

Public FAA and aviation references point to a board shape that combines FSM/TSD/OIS/CDM-style functions:

- FAA Traffic Management Initiatives guidance describes TMIs as demand/capacity tools that must be managed with the least restrictive suitable method and adjusted as conditions evolve: <https://www.faa.gov/air_traffic/publications/atpubs/foa_html/chap18_section_7.html>
- FAA TFM background material describes Traffic Situation Display map context, Flow Evaluation Areas, Flow Constrained Areas, and Flight Schedule Monitor demand/capacity modeling for airports and flow areas: <https://faa-tfm-aid.nianet.org/wp-content/uploads/Traffic-Flow-Management-Background_v3.9-1.pdf>
- FAA NAS TFM material describes monitoring airport capacity/demand, evaluating alternative approaches, publishing operational plans, and exposing advisories such as ground stops, GDPs, AFPs, routes, facility constraints, and volcanic-ash advisories: <https://seatacnoise.info/wp-content/uploads/TFM_in_the_NAS_Booklet_ca10.pdf>
- Collaborative Decision Making references describe two-way airline/ATCSCC exchange, what-if analyses, slot/delay consequences, and rapid information sharing: <https://library.unt.edu/gpo/NCARC/safetestimony/cdm.htm>
- FAA human-factors display criteria emphasize display relevancy, clarity, and usability; ATC display color research emphasizes attention, identification, segmentation, and legibility.

## Product Surface

Backend endpoint:

```text
GET  /api/tfm/board
POST /api/tfm/board
```

Frontend route:

```text
/tfm
```

The board returns:

- `airportDemand`: airport departure/arrival demand, capacity, queue, delay, runway configuration, status, source refs.
- `sectorLoad`: active aircraft, baseline capacity, workload ratio, handoff queue, frequency utilization, handoff delay, source refs.
- `activeConstraints`: active/replay TMI constraints with type, scope, target, window, delay, confidence, affected flights, source refs.
- `proposedTmis`: recommended GDP/AFP/FCA/FEA/MIT/MINIT/reroute/ground-stop/departure-metering/arrival-rate/sector-capacity review items.
- `routeAlternatives`: reroute advisory candidates and local-review alternatives with residual-risk labels.
- `impactTotals`: network-level active flights, overloaded resources, delay, proposals, affected flights, and common-operating-picture status.
- `humanFactorsNotes`: least-restrictive TMI posture, human approval requirements, source-reference drilldown expectations, and non-live-source warnings.

## Command-Center Human Factors

The board is intentionally table-first and source-first:

- Separate active constraints from proposed TMIs.
- Keep airport and sector load visible together.
- Show source mode and authorization mode before any operator acts.
- Keep every proposal human-approved; no external messages or official restrictions are sent automatically.
- Expose route alternatives as review candidates with residual risk, not hidden optimizer outputs.
- Preserve source refs so an operator can inspect the replay, demand snapshot, TMI, recommendation, or route advisory.

## Current Boundaries

- Uses local synthetic/fixture-backed SWIM/TFMS-like replay concepts unless a future authorized provider is configured.
- Does not connect to live FMDS, TFMS, SWIM, FNS, NMS, NADIN, or WMSCR.
- Capacity and delay values are representative simulation inputs, not official NAS capacity constraints.
- The UI is designed for review, simulation, evaluation, and demos; it is not a certified air traffic command-center system.

## Verification

Relevant tests:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn -q -Dtest=TfmResourceTest test
cd frontend && npm test -- --run
cd frontend && npm run build
```
