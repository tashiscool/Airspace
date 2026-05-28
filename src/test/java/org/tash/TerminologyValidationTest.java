package org.tash;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertTrue;

class TerminologyValidationTest {
    private static final List<Path> PUBLIC_OPERATOR_TEXT = List.of(
            Path.of("README.md"),
            Path.of("docs/faa-weather-gap.md"),
            Path.of("docs/SAFETY_WHITEPAPER.md"),
            Path.of("docs/DEMO_SCRIPT.md"),
            Path.of("docs/PUBLIC_REVIEW_OUTREACH.md"),
            Path.of("frontend/src/lib/feedView.ts"),
            Path.of("frontend/src/lib/viewModels.ts"),
            Path.of("frontend/src/pages/NotamsPage.tsx"),
            Path.of("frontend/src/components/WeatherVerdict.tsx"),
            Path.of("src/main/java/org/tash/extensions/product/application/AirspaceProductService.java")
    );

    private static final List<String> BANNED_OVERCLAIMS = List.of(
            "cannot depart",
            "unsafe to depart",
            "illegal",
            "airspace declares",
            "airport is in lvo",
            "lvo declared",
            "declares lvo",
            "not compliant"
    );

    @Test
    void publicOperatorTerminologyAvoidsAuthoritativeLowVisibilityOverclaims() throws IOException {
        StringBuilder failures = new StringBuilder();
        for (Path path : PUBLIC_OPERATOR_TEXT) {
            String text = Files.readString(path).toLowerCase(Locale.US);
            for (String phrase : BANNED_OVERCLAIMS) {
                if (containsPhrase(text, phrase)) {
                    failures.append(path).append(" contains banned overclaim phrase: ").append(phrase).append('\n');
                }
            }
        }
        assertTrue(failures.isEmpty(), failures.toString());
    }

    @Test
    void publicMarkdownAndFrontendCopyAvoidsBareCwapCwafOperationalClaims() throws IOException {
        List<Path> paths = new ArrayList<>();
        paths.add(Path.of("README.md"));
        try (Stream<Path> docs = Files.walk(Path.of("docs"))) {
            docs.filter(path -> path.toString().endsWith(".md"))
                    .filter(path -> !path.getFileName().toString().equals("aviation-terminology-validation.md"))
                    .filter(path -> !path.getFileName().toString().equals("aviation-terminology-gap-checklist.md"))
                    .forEach(paths::add);
        }
        try (Stream<Path> frontend = Files.walk(Path.of("frontend/src"))) {
            frontend.filter(path -> path.toString().matches(".*\\.(tsx|ts)$"))
                    .filter(path -> !path.toString().endsWith(".test.ts"))
                    .filter(path -> path.toString().contains("/pages/") || path.toString().contains("/components/"))
                    .forEach(paths::add);
        }

        StringBuilder failures = new StringBuilder();
        Pattern barePair = Pattern.compile("\\bCWAP/CWAF\\b(?![- ]?(?:style|like))", Pattern.CASE_INSENSITIVE);
        Pattern bareCwap = Pattern.compile("\\bCWAP\\b(?![- ]?style)", Pattern.CASE_INSENSITIVE);
        Pattern bareCwaf = Pattern.compile("\\bCWAF\\b(?![- ]?like)", Pattern.CASE_INSENSITIVE);
        for (Path path : paths) {
            String text = Files.readString(path);
            if (barePair.matcher(text).find() || bareCwap.matcher(text).find() || bareCwaf.matcher(text).find()) {
                failures.append(path).append(" should describe local products as CWAP-style/CWAF-like unless authoritative feeds are identified.\n");
            }
        }
        assertTrue(failures.isEmpty(), failures.toString());
    }

    @Test
    void publicCopyFramesConfidenceAsDecisionSupportNotCertifiedForecastConfidence() throws IOException {
        StringBuilder failures = new StringBuilder();
        Pattern certifiedForecastConfidence = Pattern.compile("\\b(certified|official|authoritative)\\s+meteorological\\s+forecast\\s+confidence\\b",
                Pattern.CASE_INSENSITIVE);
        for (Path path : PUBLIC_OPERATOR_TEXT) {
            String text = Files.readString(path);
            if (certifiedForecastConfidence.matcher(text).find()) {
                failures.append(path).append(" overclaims confidence as certified meteorological forecast confidence.\n");
            }
        }
        assertTrue(failures.isEmpty(), failures.toString());
    }

    private static boolean containsPhrase(String text, String phrase) {
        if (!phrase.contains(" ")) {
            return Pattern.compile("\\b" + Pattern.quote(phrase) + "\\b").matcher(text).find();
        }
        return text.contains(phrase);
    }
}
