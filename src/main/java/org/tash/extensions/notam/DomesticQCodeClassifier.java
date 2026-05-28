package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class DomesticQCodeClassifier {
    public Classification classify(DomesticNotamRecord record) {
        DomesticNotamSemanticClassification semantic = semantic(record, Collections.emptyList());
        return Classification.builder()
                .q23(semantic.getQ23())
                .q45(semantic.getQ45())
                .reason(semantic.getReducerName())
                .facilityFamily(semantic.getFacilityFamily())
                .condition(semantic.getCondition())
                .action(semantic.getAction())
                .reducerRuleId(semantic.getReducerRuleId())
                .reducerName(semantic.getReducerName())
                .warnings(semantic.getWarnings())
                .build();
    }

    public DomesticNotamSemanticClassification semantic(DomesticNotamRecord record,
                                                        List<String> recognizedContractions) {
        if (record == null || record.getText() == null) {
            return semantic(null, null, null, null, null, "DOM2.EMPTY",
                    "DOM2 empty semantic reducer", recognizedContractions, Collections.emptyList());
        }
        String keyword = upper(record.getKeyword());
        String text = upper(record.getText());
        String normalized = (" " + text.replaceAll("[^A-Z0-9]+", " ") + " ").replaceAll("\\s+", " ");
        List<String> warnings = new ArrayList<>();

        if ("NAV".equals(keyword)) {
            if (containsAny(normalized, "ILS", "MLS", "LDA", "LOC", "LLZ", "GLIDEPATH", "GLIDE", "DME")
                    && containsAny(normalized, "CAT", "NA", "NOTAUTH", "NOT", "AUTH")) {
                warnings.add("Approach landing-aid or category authorization NOTAM may affect approach capability and minima.");
                return semantic("NAV", "APPROACH_MINIMA", action(normalized), "IC", null,
                        "DOM2.NAV.APPROACH_MINIMA", "DOM2 approach minima reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "VORTAC")) {
                return semantic("NAV", "VORTAC", action(normalized), "NT", null, "DOM2.NAV.VORTAC",
                        "DOM2 navaid VORTAC reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "TACAN")) {
                return semantic("NAV", "TACAN", action(normalized), "NN", null, "DOM2.NAV.TACAN",
                        "DOM2 navaid TACAN reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "DME")) {
                return semantic("NAV", "DME", action(normalized), "ND", null, "DOM2.NAV.DME",
                        "DOM2 navaid DME reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "NDB")) {
                return semantic("NAV", "NDB", action(normalized), "NB", null, "DOM2.NAV.NDB",
                        "DOM2 navaid NDB reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "VOR")) {
                return semantic("NAV", "VOR", action(normalized), "NV", null, "DOM2.NAV.VOR",
                        "DOM2 navaid VOR reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "ILS")) {
                return semantic("NAV", "ILS", action(normalized), "IC", null, "DOM2.NAV.ILS",
                        "DOM2 runway ILS reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "MLS")) {
                return semantic("NAV", "MLS", action(normalized), "IM", null, "DOM2.NAV.MLS",
                        "DOM2 MLS reducer", recognizedContractions, warnings);
            }
            if (contains(normalized, "WAAS")) {
                return semantic("NAV", "WAAS", action(normalized), "GW", null, "DOM2.NAV.WAAS",
                        "DOM2 WAAS reducer", recognizedContractions, warnings);
            }
        }

        if (contains(normalized, "GUARD") && containsAny(normalized, "LGT", "LGTS", "LIGHT", "LIGHTS")) {
            return semantic(surfaceFamily(keyword, normalized), "GUARD_LIGHTS", action(normalized), "LS", null,
                    "DOM2.LIGHTING.GUARD", "DOM2 guard lights reducer", recognizedContractions, warnings);
        }
        if ("AIRSPACE".equals(keyword) && contains(normalized, "PJE")) {
            return semantic("AIRSPACE", "PJE", action(normalized), "WP", null, "DOM2.AIRSPACE.PJE",
                    "DOM2 parachute jumping reducer", recognizedContractions, warnings);
        }
        if ("AIRSPACE".equals(keyword) && containsAny(normalized, "AEROBATIC", "AIRDROP", "UNMANNED", "DMSTN",
                "PYROTECHNIC", "HIBAL")) {
            String condition = firstPresent(normalized, "AEROBATIC", "AIRDROP", "UNMANNED", "DMSTN", "PYROTECHNIC", "HIBAL");
            return semantic("AIRSPACE", condition, action(normalized), "WA", null, "DOM2.AIRSPACE." + condition,
                    "DOM2 special-use airspace reducer", recognizedContractions, warnings);
        }
        if ("AIRSPACE".equals(keyword) && PatternSupport.restricted(normalized)) {
            return semantic("AIRSPACE", "RESTRICTED_ZONE", action(normalized), "RR", null, "DOM2.AIRSPACE.RESTRICTED",
                    "DOM2 restricted-zone reducer", recognizedContractions, warnings);
        }
        if ("SVC".equals(keyword) && contains(normalized, "ARFF")) {
            return semantic("SVC", "ARFF", action(normalized), "FF", null, "DOM2.SVC.ARFF",
                    "DOM2 ARFF service reducer", recognizedContractions, warnings);
        }
        if (contains(normalized, "AWOS")) {
            return semantic("SVC", "AWOS", action(normalized), "FM", null, "DOM2.SVC.AWOS",
                    "DOM2 AWOS frequency reducer", recognizedContractions, warnings);
        }
        if (contains(normalized, "ASOS")) {
            return semantic("SVC", "ASOS", action(normalized), "FA", null, "DOM2.SVC.ASOS",
                    "DOM2 ASOS reducer", recognizedContractions, warnings);
        }
        if (contains(normalized, "ATIS")) {
            return semantic("SVC", "ATIS", action(normalized), "SA", null, "DOM2.SVC.ATIS",
                    "DOM2 ATIS reducer", recognizedContractions, warnings);
        }
        if (contains(normalized, "TDWR")) {
            return semantic("SVC", "TDWR", action(normalized), "WT", null, "DOM2.SVC.TDWR",
                    "DOM2 terminal Doppler weather radar reducer", recognizedContractions, warnings);
        }
        if (contains(normalized, "LLWAS")) {
            return semantic("SVC", "LLWAS", action(normalized), "WL", null, "DOM2.SVC.LLWAS",
                    "DOM2 low-level wind shear alert reducer", recognizedContractions, warnings);
        }
        if (containsAny(normalized, "RVR", "RVRM", "RVRR", "RVRT")) {
            warnings.add("Runway visual range service NOTAM may affect low-visibility departure, taxi, and coordination checks.");
            return semantic("SVC", firstPresent(normalized, "RVRT", "RVRM", "RVRR", "RVR"), action(normalized), "FT", null,
                    "DOM2.SVC.RVR", "DOM2 runway visual range reducer", recognizedContractions, warnings);
        }
        if (containsAny(normalized, "SMGCS", "LVO", "LVP")
                || (contains(normalized, "LOW") && containsAny(normalized, "VIS", "VISIBILITY"))) {
            warnings.add("Low-visibility procedure terminology retained for FAA/ICAO/airport-ops coordination review.");
            return semantic("SVC", "LOW_VISIBILITY_PROCEDURE", action(normalized), "FA", null,
                    "DOM2.SVC.LOW_VISIBILITY_PROCEDURE", "DOM2 low-visibility procedure reducer",
                    recognizedContractions, warnings);
        }
        if (contains(normalized, "PCL")) {
            String q23 = "RWY".equals(keyword) && containsAny(normalized, "LGT", "LGTS", "LIGHT", "LIGHTS") ? "LE" : null;
            return semantic(surfaceFamily(keyword, normalized), "PILOT_CONTROLLED_LIGHTING", action(normalized), q23, "XX",
                    "DOM2.LIGHTING.PCL", "DOM2 pilot-controlled lighting reducer", recognizedContractions, warnings);
        }
        if (containsAny(normalized, "ALSF", "MALS", "MALSR", "PAPI", "VASI", "REIL", "RAIL", "RCLL")) {
            warnings.add("Approach or runway lighting NOTAM may affect approach/landing minima and runway visual acquisition.");
            return semantic(surfaceFamily(keyword, normalized), "APPROACH_LIGHTING", action(normalized), "LE", null,
                    "DOM2.LIGHTING.APPROACH", "DOM2 approach/runway lighting reducer", recognizedContractions, warnings);
        }
        if (isSurfaceFamily(keyword)) {
            if (containsAny(normalized, "CLSD", "CLOSED", "UNSAFE")) {
                return semantic(surfaceFamily(keyword, normalized), "CLOSURE", action(normalized), "LC", null,
                        "DOM2.SURFACE.CLOSED", "DOM2 surface closure reducer", recognizedContractions, warnings);
            }
            if (containsAny(normalized, "PAEW", "WIP", "WARNING")) {
                return semantic(surfaceFamily(keyword, normalized), "WARNING", "WARNING", "LW", null,
                        "DOM2.SURFACE.WARNING", "DOM2 surface warning reducer", recognizedContractions, warnings);
            }
            if (containsAny(normalized, "BA", "BRAF", "BRAN", "BRAP", "MU")) {
                warnings.add("Runway surface friction/braking-action NOTAM may affect takeoff and landing performance.");
                return semantic(surfaceFamily(keyword, normalized), "FRICTION", "WARNING", "LS", null,
                        "DOM2.SURFACE.FRICTION", "DOM2 runway friction/braking reducer", recognizedContractions, warnings);
            }
            if (containsAny(normalized, "SNOW", "SN", "ICE", "SLUSH", "WET", "FROST", "BERM", "RUTS", "WINDROWS")) {
                String condition = firstPresent(normalized, "SNOW", "SN", "ICE", "SLUSH", "WET", "FROST", "BERM", "RUTS", "WINDROWS");
                return semantic(surfaceFamily(keyword, normalized), condition, action(normalized), "LS", null,
                        "DOM2.SURFACE." + condition, "DOM2 snow/surface condition reducer", recognizedContractions, warnings);
            }
            if (containsAny(normalized, "LGTS", "LIGHTS", "HIRL", "MIRL", "LIRL", "REIL", "PAPI", "VASI")) {
                return semantic(surfaceFamily(keyword, normalized), "LIGHTING", action(normalized), "LE", null,
                        "DOM2.SURFACE.LIGHTING", "DOM2 surface lighting reducer", recognizedContractions, warnings);
            }
            if (containsAny(normalized, "DSPLCD", "DISPLACED", "UNMKD", "NONSTD")) {
                return semantic(surfaceFamily(keyword, normalized), firstPresent(normalized, "DSPLCD", "DISPLACED", "UNMKD", "NONSTD"),
                        action(normalized), "LA", null, "DOM2.SURFACE.MARKING", "DOM2 runway marking/threshold reducer",
                        recognizedContractions, warnings);
            }
        }
        if ("OBST".equals(keyword) || containsAny(normalized, "TOWER", "CRANE", "BALLOON", "KITE")) {
            String condition = firstPresent(normalized, "TOWER", "CRANE", "BALLOON", "KITE");
            return semantic("OBST", condition == null ? "OBSTRUCTION" : condition, action(normalized), "OB", null,
                    "DOM2.OBST." + (condition == null ? "GENERIC" : condition),
                    "DOM2 obstruction reducer", recognizedContractions, warnings);
        }
        warnings.add("No DOM2 semantic reducer matched");
        return semantic(keyword.isEmpty() ? null : keyword, null, null, null, null, "DOM2.UNMATCHED",
                "DOM2 unmatched semantic reducer", recognizedContractions, warnings);
    }

    private DomesticNotamSemanticClassification semantic(String family, String condition, String action,
                                                         String q23, String q45, String ruleId, String ruleName,
                                                         List<String> recognizedContractions, List<String> warnings) {
        return DomesticNotamSemanticClassification.builder()
                .facilityFamily(family)
                .condition(condition)
                .action(action)
                .q23(q23)
                .q45(q45)
                .reducerRuleId(ruleId)
                .reducerName(ruleName)
                .recognizedContractions(recognizedContractions == null ? Collections.emptyList()
                        : Collections.unmodifiableList(new ArrayList<>(recognizedContractions)))
                .warnings(warnings == null ? Collections.emptyList()
                        : Collections.unmodifiableList(new ArrayList<>(warnings)))
                .build();
    }

    private String action(String text) {
        if (containsAny(text, "CLSD", "CLOSED", "UNSAFE")) {
            return "CLOSED";
        }
        if (containsAny(text, "OTS", "UNAVBL", "UNABL", "UNUSBL", "UNREL", "UNRELBL", "UNMON", "UNMNT")) {
            return "UNAVAILABLE";
        }
        if (containsAny(text, "RTS", "AVBL", "AVAILABLE")) {
            return "AVAILABLE";
        }
        if (containsAny(text, "NOW", "CHANGED")) {
            return "CHANGED";
        }
        if (containsAny(text, "PAEW", "WIP")) {
            return "WARNING";
        }
        return null;
    }

    private boolean isSurfaceFamily(String keyword) {
        return "RWY".equals(keyword) || "TWY".equals(keyword) || "APRON".equals(keyword) || "RAMP".equals(keyword);
    }

    private String surfaceFamily(String keyword, String text) {
        if ("RWY".equals(keyword) || contains(text, "RWY") || contains(text, "RWYS")) {
            return "RWY";
        }
        if ("TWY".equals(keyword) || contains(text, "TWY") || contains(text, "TAXIWAY")) {
            return "TWY";
        }
        if ("APRON".equals(keyword) || contains(text, "APRON")) {
            return "APRON";
        }
        if ("RAMP".equals(keyword) || contains(text, "RAMP")) {
            return "RAMP";
        }
        return keyword == null || keyword.isEmpty() ? null : keyword;
    }

    private String firstPresent(String text, String... tokens) {
        for (String token : tokens) {
            if (contains(text, token)) {
                return token;
            }
        }
        return null;
    }

    private boolean containsAny(String text, String... tokens) {
        for (String token : tokens) {
            if (contains(text, token)) {
                return true;
            }
        }
        return false;
    }

    private boolean contains(String text, String token) {
        return text.contains(" " + token + " ");
    }

    private String upper(String value) {
        return value == null ? "" : value.toUpperCase(Locale.US);
    }

    @Data
    @Builder
    public static class Classification {
        private String q23;
        private String q45;
        private String reason;
        private String facilityFamily;
        private String condition;
        private String action;
        private String reducerRuleId;
        private String reducerName;
        private List<String> warnings;

        private static Classification empty() {
            return Classification.builder().build();
        }
    }

    private static class PatternSupport {
        private static boolean restricted(String text) {
            return text.matches(".*\\bA?R[0-9A-Z]+(?:/[A-Z0-9]+)*\\b.*");
        }
    }
}
