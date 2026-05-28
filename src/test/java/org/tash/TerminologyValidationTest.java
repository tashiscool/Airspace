package org.tash;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

import static org.junit.jupiter.api.Assertions.assertTrue;

class TerminologyValidationTest {
    private static final List<Path> PUBLIC_OPERATOR_TEXT = List.of(
            Path.of("README.md"),
            Path.of("docs/faa-weather-gap.md"),
            Path.of("docs/SAFETY_WHITEPAPER.md"),
            Path.of("frontend/src/lib/feedView.ts"),
            Path.of("frontend/src/pages/NotamsPage.tsx"),
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

    private static boolean containsPhrase(String text, String phrase) {
        if (!phrase.contains(" ")) {
            return Pattern.compile("\\b" + Pattern.quote(phrase) + "\\b").matcher(text).find();
        }
        return text.contains(phrase);
    }
}
