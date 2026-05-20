package org.tash.extensions.carf.refdata;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PgRestoreCarfDumpExtractor implements PostgresDumpRowExtractor {
    private static final Pattern TABLE_DATA = Pattern.compile("TABLE DATA\\s+[^\\s]+\\s+([^\\s]+)");

    @Override
    public Extraction extract(Path dumpPath) throws IOException {
        if (!available()) {
            return new Extraction(false, "pg_restore is not available on PATH", Collections.emptyMap());
        }
        Process process = new ProcessBuilder("pg_restore", "-l", dumpPath.toString())
                .redirectErrorStream(true)
                .start();
        Map<String, Integer> rowCounts = new LinkedHashMap<>();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
            String line;
            while ((line = reader.readLine()) != null) {
                Matcher matcher = TABLE_DATA.matcher(line);
                if (matcher.find()) {
                    rowCounts.put(matcher.group(1), -1);
                }
            }
        }
        try {
            int exit = process.waitFor();
            if (exit != 0) {
                return new Extraction(false, "pg_restore -l failed with exit code " + exit, rowCounts);
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            return new Extraction(false, "pg_restore interrupted", rowCounts);
        }
        return new Extraction(true,
                "pg_restore is available; table data entries are listed, row counts require restore/COPY execution",
                rowCounts);
    }

    private boolean available() {
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
}
