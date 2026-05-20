package org.tash.extensions.feed;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * CSV-like local reference-data adapter.
 *
 * Format:
 * type,identifier,latitude,longitude,altitudeFeet,source,sourceVersion
 */
public class LocalReferenceDataSyncAdapter implements ReferenceDataSyncAdapter {
    @Override
    public ReferenceDataSyncResult preview(String payload) {
        List<ReferenceDataImportRecord> records = new ArrayList<>();
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        Set<String> keys = new HashSet<>();
        if (payload == null || payload.trim().isEmpty()) {
            errors.add("Reference-data payload is empty");
            return ReferenceDataSyncResult.builder().accepted(false).records(records).warnings(warnings).errors(errors).build();
        }
        String[] lines = payload.split("\\R");
        for (int i = 0; i < lines.length; i++) {
            String line = lines[i].trim();
            if (line.isEmpty() || line.startsWith("#") || (i == 0 && line.toLowerCase(Locale.US).contains("identifier"))) {
                continue;
            }
            String[] cells = line.split(",");
            if (cells.length < 4) {
                errors.add("Line " + (i + 1) + " has fewer than four columns");
                continue;
            }
            try {
                String type = upper(cells[0], "FIX");
                String identifier = upper(cells[1], null);
                if (identifier == null) {
                    errors.add("Line " + (i + 1) + " missing identifier");
                    continue;
                }
                String key = type + ":" + identifier;
                if (!keys.add(key)) {
                    warnings.add("Duplicate reference point in import: " + key);
                }
                records.add(ReferenceDataImportRecord.builder()
                        .pointType(type)
                        .identifier(identifier)
                        .latitude(Double.parseDouble(cells[2].trim()))
                        .longitude(Double.parseDouble(cells[3].trim()))
                        .altitudeFeet(cells.length > 4 && !cells[4].trim().isEmpty() ? Double.parseDouble(cells[4].trim()) : 0.0)
                        .source(cells.length > 5 && !cells[5].trim().isEmpty() ? cells[5].trim() : "reference-sync")
                        .sourceVersion(cells.length > 6 && !cells[6].trim().isEmpty() ? cells[6].trim() : "local")
                        .metadataJson("{\"sourceVersion\":\"" + (cells.length > 6 ? cells[6].trim() : "local") + "\"}")
                        .build());
            } catch (NumberFormatException e) {
                errors.add("Line " + (i + 1) + " has invalid latitude/longitude/altitude: " + e.getMessage());
            }
        }
        return ReferenceDataSyncResult.builder()
                .accepted(errors.isEmpty())
                .records(records)
                .warnings(warnings)
                .errors(errors)
                .build();
    }

    private String upper(String value, String fallback) {
        if (value == null || value.trim().isEmpty()) {
            return fallback;
        }
        return value.trim().toUpperCase(Locale.US);
    }
}
