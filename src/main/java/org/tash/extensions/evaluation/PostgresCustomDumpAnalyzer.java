package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PostgresCustomDumpAnalyzer {
    private static final byte[] MAGIC = new byte[]{'P', 'G', 'D', 'M', 'P'};
    private static final Pattern CREATE_TABLE = Pattern.compile("CREATE TABLE \"([^\"]+)\"");
    private static final Pattern COPY_TABLE = Pattern.compile("COPY \"([^\"]+)\"");
    private static final Pattern DATABASE = Pattern.compile("CREATE DATABASE \"([^\"]+)\"");
    private static final Pattern SEQUENCE_VALUE = Pattern.compile("SELECT pg_catalog\\.setval\\('\"([^\"]+)\"',\\s*(\\d+),");
    private final PostgresDumpRowExtractor rowExtractor;

    public PostgresCustomDumpAnalyzer() {
        this(new BlockedPostgresDumpRowExtractor());
    }

    public PostgresCustomDumpAnalyzer(PostgresDumpRowExtractor rowExtractor) {
        this.rowExtractor = rowExtractor;
    }

    public Analysis analyze(Path path) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        List<String> strings = new LegacyArtifactTextExtractor().printableStrings(path, 5);
        Set<String> tables = new LinkedHashSet<>();
        Set<String> copyTables = new LinkedHashSet<>();
        Set<String> relevantTables = new LinkedHashSet<>();
        Set<String> sequenceValues = new LinkedHashSet<>();
        String databaseName = null;

        for (String text : strings) {
            databaseName = firstMatch(DATABASE, text, databaseName);
            addMatches(CREATE_TABLE, text, tables);
            addMatches(COPY_TABLE, text, copyTables);
            Matcher sequence = SEQUENCE_VALUE.matcher(text);
            while (sequence.find()) {
                sequenceValues.add(sequence.group(1) + "=" + sequence.group(2));
            }
        }

        for (String table : tables) {
            String normalized = table.toLowerCase();
            if (normalized.contains("airspace")
                    || normalized.contains("separation")
                    || normalized.contains("navaid")
                    || normalized.contains("route")
                    || normalized.contains("reservation")
                    || normalized.contains("message")
                    || normalized.contains("mission")
                    || normalized.contains("notam")) {
                relevantTables.add(table);
            }
        }

        PostgresDumpRowExtractor.Extraction extraction = rowExtractor.extract(path);

        return Analysis.builder()
                .path(path)
                .postgresCustomDump(hasMagic(bytes))
                .pgRestoreAvailable(isPgRestoreAvailable())
                .rowExtractionAvailable(extraction.isAvailable())
                .rowExtractionDiagnostic(extraction.getDiagnostic())
                .databaseName(databaseName)
                .tables(tables)
                .copyTables(copyTables)
                .relevantTables(relevantTables)
                .sequenceValues(sequenceValues)
                .build();
    }

    private boolean isPgRestoreAvailable() {
        try {
            Process process = new ProcessBuilder("pg_restore", "--version")
                    .redirectErrorStream(true)
                    .start();
            return process.waitFor() == 0;
        } catch (IOException ex) {
            return false;
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            return false;
        }
    }

    private boolean hasMagic(byte[] bytes) {
        if (bytes.length < MAGIC.length) {
            return false;
        }
        for (int i = 0; i < MAGIC.length; i++) {
            if (bytes[i] != MAGIC[i]) {
                return false;
            }
        }
        return true;
    }

    private String firstMatch(Pattern pattern, String text, String current) {
        if (current != null) {
            return current;
        }
        Matcher matcher = pattern.matcher(text);
        return matcher.find() ? matcher.group(1) : null;
    }

    private void addMatches(Pattern pattern, String text, Set<String> target) {
        Matcher matcher = pattern.matcher(text);
        while (matcher.find()) {
            target.add(matcher.group(1));
        }
    }

    @Data
    @Builder
    public static class Analysis {
        private Path path;
        private boolean postgresCustomDump;
        private boolean pgRestoreAvailable;
        private boolean rowExtractionAvailable;
        private String rowExtractionDiagnostic;
        private String databaseName;
        private Set<String> tables;
        private Set<String> copyTables;
        private Set<String> relevantTables;
        private Set<String> sequenceValues;
    }
}
