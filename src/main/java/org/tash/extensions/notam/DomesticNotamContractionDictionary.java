package org.tash.extensions.notam;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DomesticNotamContractionDictionary {
    private static final Pattern YCC_CONTRACTION =
            Pattern.compile("^'([^']+)'\\s+([A-Za-z][A-Za-z0-9_]*)\\s*;");
    private static final Map<String, String> DEFAULT_CONTRACTIONS = build();
    private final Map<String, String> contractions;

    public DomesticNotamContractionDictionary() {
        this(DEFAULT_CONTRACTIONS);
    }

    public DomesticNotamContractionDictionary(Map<String, String> contractions) {
        Map<String, String> normalized = new HashMap<>();
        for (Map.Entry<String, String> entry : contractions.entrySet()) {
            put(normalized, entry.getKey(), entry.getValue());
        }
        this.contractions = Collections.unmodifiableMap(normalized);
    }

    public Optional<String> classify(String token) {
        if (token == null) {
            return Optional.empty();
        }
        return Optional.ofNullable(contractions.get(normalize(token)));
    }

    public Map<String, String> entries() {
        return contractions;
    }

    public static DomesticNotamContractionDictionary fromLegacyYcc(Path path) throws IOException {
        Map<String, String> values = new HashMap<>(DEFAULT_CONTRACTIONS);
        for (String rawLine : Files.readAllLines(path)) {
            String line = rawLine.replaceFirst("//.*$", "").trim();
            Matcher matcher = YCC_CONTRACTION.matcher(line);
            if (matcher.find()) {
                put(values, matcher.group(1), matcher.group(2).toLowerCase(Locale.US));
            }
        }
        return new DomesticNotamContractionDictionary(values);
    }

    private static Map<String, String> build() {
        Map<String, String> values = new HashMap<>();
        put(values, "APCH", "approach");
        put(values, "APPROACH", "approach");
        put(values, "UNAVBL", "unavailable");
        put(values, "UNABL", "unavailable");
        put(values, "TAXILANE", "taxilane");
        put(values, "TAXILANES", "taxilane");
        put(values, "TAXIS", "taxi");
        put(values, "SFC", "surface");
        put(values, "DMSNT", "dismantled");
        put(values, "DMSTN", "dismantled");
        put(values, "ACFT", "aircraft");
        put(values, "LGTS", "lights");
        put(values, "LGTD", "lighted");
        put(values, "UNMON", "unmonitored");
        put(values, "UNMONITORED", "unmonitored");
        put(values, "PAEW", "personnel and equipment working");
        put(values, "DISABLED", "disabled");
        put(values, "MOORED", "moored");
        put(values, "SHIP", "ship");
        put(values, "SKI", "ski");
        put(values, "STRIP", "strip");
        put(values, "TURNAROUND", "turnaround");
        put(values, "TURNAROUNDS", "turnaround");
        put(values, "SNOW", "snow");
        put(values, "GUARD", "guard");
        put(values, "FT", "feet");
        put(values, "RVR", "runway visual range");
        put(values, "RVRM", "runway visual range midpoint");
        put(values, "RVRR", "runway visual range rollout");
        put(values, "RVRT", "runway visual range touchdown");
        put(values, "VIS", "visibility");
        put(values, "SMGCS", "surface movement guidance and control system");
        put(values, "LVO", "low visibility operations");
        put(values, "LVP", "low visibility procedures");
        put(values, "APRON", "apron");
        put(values, "RAMP", "ramp");
        return values;
    }

    private static void put(Map<String, String> values, String token, String classification) {
        values.put(normalize(token), classification);
    }

    private static String normalize(String token) {
        return token.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }
}
