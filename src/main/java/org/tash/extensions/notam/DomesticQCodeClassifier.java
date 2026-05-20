package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.Locale;

public class DomesticQCodeClassifier {
    public Classification classify(DomesticNotamRecord record) {
        if (record == null || record.getText() == null) {
            return Classification.empty();
        }
        String keyword = upper(record.getKeyword());
        String text = upper(record.getText());
        String normalized = (" " + text.replaceAll("[^A-Z0-9]+", " ") + " ").replaceAll("\\s+", " ");

        if ("NAV".equals(keyword)) {
            if (contains(normalized, "VORTAC")) {
                return q23("NT", "DOM2 navaid VORTAC reducer");
            }
            if (contains(normalized, "TACAN")) {
                return q23("NN", "DOM2 navaid TACAN reducer");
            }
            if (contains(normalized, "DME")) {
                return q23("ND", "DOM2 navaid DME reducer");
            }
            if (contains(normalized, "NDB")) {
                return q23("NB", "DOM2 navaid NDB reducer");
            }
            if (contains(normalized, "VOR")) {
                return q23("NV", "DOM2 navaid VOR reducer");
            }
        }

        if (contains(normalized, "GUARD") && containsAny(normalized, "LGT", "LGTS", "LIGHT", "LIGHTS")) {
            return q23("LS", "DOM2 guard lights reducer");
        }
        if ("AIRSPACE".equals(keyword) && contains(normalized, "PJE")) {
            return q23("WP", "DOM2 parachute jumping reducer");
        }
        if ("SVC".equals(keyword) && contains(normalized, "ARFF")) {
            return q23("FF", "DOM2 ARFF service reducer");
        }
        if (contains(normalized, "AWOS")) {
            return q23("FM", "DOM2 AWOS frequency reducer");
        }
        if (contains(normalized, "ATIS")) {
            return q23("SA", "DOM2 ATIS reducer");
        }
        if (contains(normalized, "RVR")) {
            return q23("FT", "DOM2 runway visual range reducer");
        }
        if (contains(normalized, "PCL")) {
            Classification classification = q45("XX", "DOM2 pilot-controlled lighting reducer");
            if ("RWY".equals(keyword) && containsAny(normalized, "LGT", "LGTS", "LIGHT", "LIGHTS")) {
                classification.q23 = "LE";
            }
            return classification;
        }
        return Classification.empty();
    }

    private Classification q23(String value, String reason) {
        return Classification.builder().q23(value).reason(reason).build();
    }

    private Classification q45(String value, String reason) {
        return Classification.builder().q45(value).reason(reason).build();
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

        private static Classification empty() {
            return Classification.builder().build();
        }
    }
}
