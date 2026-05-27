# Weather Pattern Mapping

Airspace maps aviation weather as deterministic, source-cited operational patterns. The goal is not to certify a new forecast model; it is to make official aviation weather products inspectable in the same route, altitude, time, NOTAM, CARF/ALTRV, PIREP, deconfliction, and pilot-brief context used by the workbench.

## Product Families

- Live polling is configuration-gated with `airspace.weather.live.enabled=false` by default.
- `LiveAviationWeatherAdapter` targets the NOAA/AWC Data API base URL configured by `airspace.weather.awc.base-url`.
- The adapter is designed for METAR/SPECI, TAF, PIREP/AIREP, SIGMET, G-AIRMET/AIRMET, CWA, and TCF-style products.
- GeoJSON is preferred when available. Raw aviation text and feature properties are retained with source URL and source refs.

## Pattern Model

Weather products normalize into deterministic pattern types:

- `CONVECTION`
- `TURBULENCE`
- `ICING`
- `WIND_SHEAR`
- `VOLCANIC_ASH`
- `CEILING_VISIBILITY`
- `PRECIPITATION`
- `PIREP_CLUSTER`
- `TERMINAL_FORECAST`
- `GENERIC_ADVISORY`

Each pattern preserves geometry, altitude band, valid time window, forecast hour, movement, severity, confidence, freshness, source refs, retained raw text, and diagnostics when available.

## Geometry Rules

- Coordinate-bearing SIGMET/AIRMET/CWA/TCF-style products become map features.
- PIREPs become point observations when location is available.
- METAR/TAF products without route geometry remain station/time guidance artifacts.
- Airspace does not invent polygons for products that do not provide usable geometry.

## Time, Altitude, And Movement

- Map features expose valid start/end, forecast hour, altitude bounds, movement bearing/speed, freshness, confidence, and severity.
- The workbench map can filter by forecast hour, altitude, freshness, confidence, and source family.
- Movement is shown as operational metadata and can be toggled in the map readout.

## Events And Route Sampling

- Weather events group patterns by deterministic hazard family: convection, turbulence, icing, volcanic ash, PIREP clusters, ceiling/visibility, precipitation, and generic advisories.
- Route sampling compares a route trajectory/corridor against weather patterns using geometry proximity plus time and altitude overlap.
- Route sampling output is advisory and source-cited; the existing deterministic route-impact engine remains authoritative for mission guidance.

## Limits

- This is a production-grade prototype, not certified cockpit, dispatch, or FAA operational deployment software.
- Live AWC polling requires explicit configuration and should respect local rate limits, polling intervals, and bounded result windows.
- Historical calibration, sector-demand validation, and certified model evaluation remain separate work.
