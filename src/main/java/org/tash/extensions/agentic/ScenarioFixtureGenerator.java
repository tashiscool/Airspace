package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@ApplicationScoped
public class ScenarioFixtureGenerator {
    public ScenarioFixtureBundle generate(ScenarioFixtureRequest request) {
        ScenarioFixtureRequest safe = request == null ? new ScenarioFixtureRequest() : request;
        String type = normalize(safe.getScenarioType());
        String mission = safe.getMissionNumber() == null || safe.getMissionNumber().trim().isEmpty()
                ? "NXGEN-" + type
                : safe.getMissionNumber().trim();
        List<List<Double>> route = Arrays.asList(
                Arrays.asList(30.0, -150.5, 24000.0),
                Arrays.asList(30.5, -149.8, 25000.0),
                Arrays.asList(31.0, -149.0, 26000.0));
        Map<String, String> expected = new LinkedHashMap<>();
        expected.put("scenarioType", type);
        expected.put("missionNumber", mission);
        expected.put("certification", "fixture-only");
        List<String> weather = new ArrayList<>();
        List<String> pireps = new ArrayList<>();
        List<String> notams = new ArrayList<>();
        switch (type) {
            case "ICING_ALTITUDE_BAND":
                weather.add("AIRMET ZULU VALID 200000/200600 FROM 3000N15000W TO 3100N14900W MOD ICE BLW FL180 MOV E 15KT");
                pireps.add("UA /OV 3000N15000W/TM 2005/FL250/TP B738/IC NEG/RM ABOVE ICING LAYER");
                expected.put("expectedAction", "MONITOR_OR_CLEAR_ABOVE_LAYER");
                break;
            case "PIREP_CLUSTER":
                pireps.add("UUA /OV 3000N15000W/TM 2000/FL250/TP B738/TB SEV/RM URGENT ROUTE IMPACT");
                pireps.add("UA /OV 3050N14950W/TM 2010/FL260/TP A320/TB MOD-SEV/RM CLUSTERED TURB");
                expected.put("expectedAction", "CAUTION_OR_ALTITUDE_CHANGE");
                break;
            case "NOTAM_WEATHER_COMPOUND":
                weather.add("SIGMET COMPOUND VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS TOP FL450 INTSF");
                notams.add("!FDC COMPOUND NOTAM AIRSPACE CLSD WI AN AREA 3000N15000W-3100N14900W SFC-FL260");
                expected.put("expectedAction", "REROUTE_OR_BLOCKED");
                break;
            case "VIABLE_REROUTE":
                weather.add("CWAP REROUTE VALID 200000/200400 WITHIN 35NM OF 3030N14950W TOP FL430 MOV E 20KT GROWING");
                expected.put("expectedAction", "REROUTE_WITH_CANDIDATE");
                break;
            case "NO_VIABLE_REROUTE":
                weather.add("SIGMET BLOCK VALID 200000/200800 BOUNDED BY 2950N15100W 3150N15100W 3150N14800W 2950N14800W EMBD TS TOP FL500 STNR");
                notams.add("!FDC BLOCK NOTAM AIRSPACE CLSD WI AN AREA 2950N15100W-3150N14800W SFC-FL600");
                expected.put("expectedAction", "BLOCKED");
                break;
            case "MALFORMED_RETAINED":
                weather.add("SIGMET MISSING VALID BUT NO GEOMETRY SEV TURB");
                notams.add("!FDC BAD NOTAM AIRSPACE TEXT WITHOUT POSITION");
                expected.put("expectedAction", "REVIEW_WITH_DIAGNOSTICS");
                break;
            case "SEVERE_CONVECTION_BLOCKS_ROUTE":
            default:
                weather.add("SIGMET CONV VALID 200000/200400 FROM 3000N15000W TO 3100N14900W EMBD TS MOV E 25KT TOP FL450 INTSF");
                pireps.add("UA /OV 3030N14950W/TM 2005/FL250/TP B738/TB MOD/RM CONVECTIVE ROUTE IMPACT");
                expected.put("expectedAction", "REROUTE_OR_BLOCKED");
                break;
        }
        if (safe.isIncludeMalformedInputs()) {
            weather.add("TAF CORRUPT WITHOUT STATION OR TIME");
            expected.put("malformedRetained", "true");
        }
        String carf = "A. " + mission + "\n"
                + "B. 1KC135/M\n"
                + "C. JFK 200000Z\n"
                + "D. 3000N15000W 3050N14950W 3100N14900W\n"
                + "E. DOV\n"
                + "F. FL240-FL280\n"
                + "G. GENERATED NEXTGEN SCENARIO " + type;
        List<String> usns = new ArrayList<>();
        weather.forEach(message -> usns.add("USNS " + type + " WX " + message));
        notams.forEach(message -> usns.add("USNS " + type + " NOTAM " + message));
        pireps.forEach(message -> usns.add("USNS " + type + " PIREP " + message));
        return ScenarioFixtureBundle.builder()
                .id(AgentSupport.id("scenario", type + ":" + mission))
                .scenarioType(type)
                .missionNumber(mission)
                .carfAltrv(carf)
                .usnsMessages(usns)
                .weatherMessages(weather)
                .pireps(pireps)
                .notams(notams)
                .route(route)
                .expectedSummary(expected)
                .build();
    }

    private String normalize(String type) {
        if (type == null || type.trim().isEmpty()) {
            return "SEVERE_CONVECTION_BLOCKS_ROUTE";
        }
        return type.trim().replace('-', '_').replace(' ', '_').toUpperCase(Locale.US);
    }
}
