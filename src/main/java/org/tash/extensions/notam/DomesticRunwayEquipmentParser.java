package org.tash.extensions.notam;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DomesticRunwayEquipmentParser {
    private static final Pattern RUNWAY = Pattern.compile("\\bRWY\\s+([A-Z0-9/]+)\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern DISTANCE_BEFORE_EQUIPMENT =
            Pattern.compile("\\b(\\d{1,2},?\\d{3}|\\d{4,5})(?:\\s*/\\s*(\\d{1,2},?\\d{3}|\\d{4,5}))?\\s*(?:FT\\s+)?(?:DIST(?:ANCE)?\\s+)?REMAINING\\b",
                    Pattern.CASE_INSENSITIVE);
    private static final Pattern LISTED_MARKERS =
            Pattern.compile("\\b(?:MARKERS?|SIGNS?)\\s+((?:\\d{1,2},?\\d{3}\\s*,?\\s*)+)\\s+HAVE\\s+BEEN\\s+REMOVED\\b",
                    Pattern.CASE_INSENSITIVE);

    private final DomesticNotamParser domesticNotamParser;

    public DomesticRunwayEquipmentParser() {
        this(new DomesticNotamParser());
    }

    public DomesticRunwayEquipmentParser(DomesticNotamParser domesticNotamParser) {
        this.domesticNotamParser = domesticNotamParser;
    }

    public DomesticRunwayEquipmentStatus parse(String rawText) {
        DomesticNotamRecord record = domesticNotamParser.parse(rawText);
        if (!"RWY".equals(record.getKeyword())) {
            throw new IllegalArgumentException("Domestic NOTAM is not a runway equipment status");
        }

        String text = record.getText() == null ? "" : record.getText();
        Set<DomesticRunwayEquipmentStatus.Equipment> equipment = equipment(text);
        Set<DomesticRunwayEquipmentStatus.Status> statuses = statuses(text);
        if (equipment.isEmpty() || statuses.isEmpty()) {
            throw new IllegalArgumentException("Runway NOTAM is missing tracked equipment/status");
        }

        return DomesticRunwayEquipmentStatus.builder()
                .type(record.getType())
                .accountability(record.getAccountability())
                .notamNumber(record.getNotamNumber())
                .location(record.getLocation())
                .runway(runway(record, text))
                .distancesFeet(distances(text))
                .equipment(equipment)
                .statuses(statuses)
                .effectiveStart(record.getEffectiveStart())
                .effectiveEnd(record.getEffectiveEnd())
                .description(record.getText())
                .build();
    }

    private String runway(DomesticNotamRecord record, String text) {
        Matcher matcher = RUNWAY.matcher(record.getKeyword() + " " + text);
        return matcher.find() ? matcher.group(1) : null;
    }

    private List<Integer> distances(String text) {
        List<Integer> distances = new ArrayList<>();
        Matcher inline = DISTANCE_BEFORE_EQUIPMENT.matcher(text);
        while (inline.find()) {
            addDistance(distances, inline.group(1));
            addDistance(distances, inline.group(2));
        }

        Matcher listed = LISTED_MARKERS.matcher(text);
        if (listed.find()) {
            for (String value : listed.group(1).split(",")) {
                addDistance(distances, value);
            }
        }
        return distances;
    }

    private void addDistance(List<Integer> distances, String value) {
        if (value == null) {
            return;
        }
        int distance = Integer.parseInt(value.replaceAll("[^0-9]", ""));
        if (!distances.contains(distance)) {
            distances.add(distance);
        }
    }

    private Set<DomesticRunwayEquipmentStatus.Equipment> equipment(String text) {
        String upper = text.toUpperCase(Locale.US);
        Set<DomesticRunwayEquipmentStatus.Equipment> equipment = new LinkedHashSet<>();
        if (upper.matches(".*\\bDIST(?:ANCE)?\\s+REMAINING\\s+SIGNS?\\b.*")
                || upper.matches(".*\\bREMAINING\\s+SIGNS?\\b.*")) {
            equipment.add(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_SIGN);
        }
        if (upper.matches(".*\\bDIST(?:ANCE)?\\s+REMAINING\\s+MARKERS?\\b.*")
                || upper.matches(".*\\bREMAINING\\s+MARKERS?\\b.*")) {
            equipment.add(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_MARKER);
        }
        if (upper.matches(".*\\bRWY\\s+LGTS?\\b.*")) {
            equipment.add(DomesticRunwayEquipmentStatus.Equipment.RUNWAY_LIGHTS);
        }
        if (upper.matches(".*\\bMARKINGS\\b.*")) {
            equipment.add(DomesticRunwayEquipmentStatus.Equipment.RUNWAY_MARKINGS);
        }
        return equipment;
    }

    private Set<DomesticRunwayEquipmentStatus.Status> statuses(String text) {
        String upper = text.toUpperCase(Locale.US);
        Set<DomesticRunwayEquipmentStatus.Status> statuses = new LinkedHashSet<>();
        if (upper.matches(".*\\bOTS\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.OUT_OF_SERVICE);
        }
        if (upper.matches(".*\\bUNLGTD\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.UNLIGHTED);
        }
        if (upper.matches(".*\\bUNLIT\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.UNLIT);
        }
        if (upper.matches(".*\\bOBSC\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.OBSCURED);
        }
        if (upper.matches(".*\\bMISSING\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.MISSING);
        }
        if (upper.matches(".*\\bREMOVED\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.REMOVED);
        }
        if (upper.matches(".*\\bNONSTD\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.NONSTANDARD);
        }
        if (upper.matches(".*\\bNON[- ]?CONFORMING\\b.*")) {
            statuses.add(DomesticRunwayEquipmentStatus.Status.NONCONFORMING);
        }
        return statuses;
    }
}
