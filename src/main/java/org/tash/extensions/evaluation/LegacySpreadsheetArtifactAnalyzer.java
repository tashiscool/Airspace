package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

public class LegacySpreadsheetArtifactAnalyzer {
    private static final byte[] CFB_MAGIC = new byte[]{
            (byte) 0xD0, (byte) 0xCF, 0x11, (byte) 0xE0, (byte) 0xA1, (byte) 0xB1, 0x1A, (byte) 0xE1
    };
    private static final Pattern SHEET_NAME = Pattern.compile("(?i)(sheet\\d+|test\\d+|[^!]{1,40}!Print_Area)");
    private static final Pattern PHONE_LIKE = Pattern.compile(".*\\d{10,15}.*");
    private static final Pattern FACILITY_LIKE = Pattern.compile("(?i).*(\\bZ[A-Z]{2}\\s*TMU\\b|\\bCARF\\b|\\bRANGE\\b|\\bSHIFT MANAGER\\b|\\bVACAPES\\b|\\bBARKING SANDS\\b).*");
    private static final Pattern EXECUTABLE_CARF = Pattern.compile("(?i).*(\\bETD\\b|\\bAVANA\\b|\\bFL\\d{3}B\\d{3}\\b|\\bNOTAM\\b|^![A-Z0-9]{3}\\b).*");

    public Analysis analyze(Path path) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        List<String> strings = new LegacyArtifactTextExtractor().printableStrings(path, 4);
        Set<String> sheetNames = new LinkedHashSet<>();
        Set<String> contactTokens = new LinkedHashSet<>();
        Set<String> executableRows = new LinkedHashSet<>();

        for (String text : strings) {
            for (String rawLine : text.replace('\r', '\n').split("\\n")) {
                String line = rawLine.trim().replaceAll("\\s+", " ");
                if (line.isEmpty()) {
                    continue;
                }
                if (SHEET_NAME.matcher(line).matches()) {
                    sheetNames.add(line);
                }
                if (PHONE_LIKE.matcher(line).matches() || FACILITY_LIKE.matcher(line).matches()) {
                    contactTokens.add(line);
                }
                if (EXECUTABLE_CARF.matcher(line).matches()) {
                    executableRows.add(line);
                }
            }
        }

        return Analysis.builder()
                .path(path)
                .compoundDocument(hasCompoundDocumentMagic(bytes))
                .sheetNames(sheetNames)
                .contactTokens(contactTokens)
                .executableRows(executableRows)
                .referenceOnly(executableRows.isEmpty())
                .build();
    }

    private boolean hasCompoundDocumentMagic(byte[] bytes) {
        if (bytes.length < CFB_MAGIC.length) {
            return false;
        }
        for (int i = 0; i < CFB_MAGIC.length; i++) {
            if (bytes[i] != CFB_MAGIC[i]) {
                return false;
            }
        }
        return true;
    }

    @Data
    @Builder
    public static class Analysis {
        private Path path;
        private boolean compoundDocument;
        private Set<String> sheetNames;
        private Set<String> contactTokens;
        private Set<String> executableRows;
        private boolean referenceOnly;
    }
}
