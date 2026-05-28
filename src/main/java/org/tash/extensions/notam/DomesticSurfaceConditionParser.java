package org.tash.extensions.notam;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DomesticSurfaceConditionParser {
    private static final Set<String> SURFACE_KEYWORDS = new HashSet<>(Arrays.asList(
            "RWY", "TWY", "RAMP", "APRON", "AD"
    ));
    private static final Set<String> CONDITION_MARKERS = new HashSet<>(Arrays.asList(
            "THN", "PTCHY", "LOOSE", "COMPACT", "COMPACTED", "PACKED", "SN", "SNOW", "ICE",
            "FROST", "SLUSH", "IR", "SIR", "LSR", "PSR", "WSR", "WTR", "BERM", "BERMS", "PILE",
            "PILES", "SNOWPILE", "SNOWPILES", "TURNAROUND", "TURNAROUNDS", "BA", "BRAF", "BRAN",
            "BRAP", "MU"
    ));
    private static final Pattern HEIGHT_BEFORE_BERM =
            Pattern.compile("(\\d+(?:\\.\\d+)?|1/2|1/4)\\s*(IN|FT)\\s+BERMS?\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern HEIGHT_BEFORE_PILE =
            Pattern.compile("(\\d+(?:\\.\\d+)?|1/2|1/4)\\s*(IN|FT)\\s+(?:SNW\\s+|SNOW\\s*)?PILES?\\b",
                    Pattern.CASE_INSENSITIVE);

    private final DomesticNotamParser domesticNotamParser;

    public DomesticSurfaceConditionParser() {
        this(new DomesticNotamParser());
    }

    public DomesticSurfaceConditionParser(DomesticNotamParser domesticNotamParser) {
        this.domesticNotamParser = domesticNotamParser;
    }

    public DomesticSurfaceCondition parse(String rawText) {
        DomesticNotamRecord record = domesticNotamParser.parse(rawText);
        if (!SURFACE_KEYWORDS.contains(record.getKeyword())) {
            throw new IllegalArgumentException("Domestic NOTAM is not a supported surface condition");
        }

        String body = record.getText() == null ? "" : record.getText();
        Set<DomesticSurfaceCondition.Hazard> hazards = hazards(body);
        if (hazards.isEmpty()) {
            throw new IllegalArgumentException("Domestic surface NOTAM does not contain a tracked condition");
        }

        return DomesticSurfaceCondition.builder()
                .type(record.getType())
                .accountability(record.getAccountability())
                .notamNumber(record.getNotamNumber())
                .location(record.getLocation())
                .keyword(record.getKeyword())
                .affectedSurface(affectedSurface(body))
                .hazards(hazards)
                .bermHeightInches(heightBefore(body, HEIGHT_BEFORE_BERM, true))
                .snowPileHeightFeet(heightBefore(body, HEIGHT_BEFORE_PILE, false))
                .effectiveStart(record.getEffectiveStart())
                .effectiveEnd(record.getEffectiveEnd())
                .description(record.getText())
                .build();
    }

    private Set<DomesticSurfaceCondition.Hazard> hazards(String body) {
        String text = body.toUpperCase(Locale.US);
        Set<DomesticSurfaceCondition.Hazard> hazards = new LinkedHashSet<>();
        if (text.matches(".*\\bBERMS?\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.BERM);
        }
        if (text.matches(".*\\b(?:SNW\\s+|SNOW\\s+)?PILES?\\b.*")
                || text.matches(".*\\bSNOWPILES?\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.SNOW_PILE);
        }
        if (text.matches(".*\\bTURNAROUNDS?\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.TURNAROUND);
        }
        if (text.matches(".*\\b(?:SN|SNOW)\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.SNOW);
        }
        if (text.matches(".*\\b(?:ICE|IR)\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.ICE);
        }
        if (text.matches(".*\\bSLUSH\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.SLUSH);
        }
        if (text.matches(".*\\bFROST\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.FROST);
        }
        if (text.matches(".*\\b(?:WTR|WATER)\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.WATER);
        }
        if (text.matches(".*\\bDEICED\\s+LIQUID\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.DEICED_LIQUID);
        }
        if (text.matches(".*\\b(?:BA|BRAF|BRAN|BRAP)\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.BRAKING_ACTION);
        }
        if (text.matches(".*\\bMU\\b.*")) {
            hazards.add(DomesticSurfaceCondition.Hazard.FRICTION_COEFFICIENT);
        }
        return hazards;
    }

    private String affectedSurface(String body) {
        String[] tokens = body.split("\\s+");
        StringBuilder surface = new StringBuilder();
        for (int i = 0; i < tokens.length; i++) {
            String token = tokens[i];
            if (CONDITION_MARKERS.contains(token.toUpperCase(Locale.US))) {
                break;
            }
            if (isMeasurementBeforeCondition(tokens, i)) {
                break;
            }
            if (surface.length() > 0) {
                surface.append(' ');
            }
            surface.append(token);
        }
        return surface.length() == 0 ? null : surface.toString();
    }

    private boolean isMeasurementBeforeCondition(String[] tokens, int index) {
        if (index + 2 >= tokens.length) {
            return false;
        }
        String token = tokens[index];
        String unit = tokens[index + 1].toUpperCase(Locale.US);
        String condition = tokens[index + 2].toUpperCase(Locale.US);
        return token.matches("\\d+(?:\\.\\d+)?|1/2|1/4")
                && ("IN".equals(unit) || "FT".equals(unit))
                && CONDITION_MARKERS.contains(condition);
    }

    private Double heightBefore(String body, Pattern pattern, boolean outputInches) {
        Matcher matcher = pattern.matcher(body);
        if (!matcher.find()) {
            return null;
        }
        double value = parseNumber(matcher.group(1));
        String unit = matcher.group(2).toUpperCase(Locale.US);
        if (outputInches) {
            return "FT".equals(unit) ? value * 12.0 : value;
        }
        return "IN".equals(unit) ? value / 12.0 : value;
    }

    private double parseNumber(String value) {
        if ("1/2".equals(value)) {
            return 0.5;
        }
        if ("1/4".equals(value)) {
            return 0.25;
        }
        return Double.parseDouble(value);
    }
}
