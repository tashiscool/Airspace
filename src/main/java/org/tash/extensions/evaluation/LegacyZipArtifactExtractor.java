package org.tash.extensions.evaluation;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public class LegacyZipArtifactExtractor {
    public Map<String, byte[]> entries(Path zipPath) throws IOException {
        Map<String, byte[]> entries = new LinkedHashMap<>();
        try (ZipInputStream input = new ZipInputStream(Files.newInputStream(zipPath))) {
            ZipEntry entry;
            while ((entry = input.getNextEntry()) != null) {
                if (entry.isDirectory()) {
                    continue;
                }
                entries.put(entry.getName(), readEntry(input));
            }
        }
        return entries;
    }

    public Map<String, String> textEntries(Path zipPath) throws IOException {
        return textEntries(zipPath, StandardCharsets.UTF_8);
    }

    public Map<String, String> textEntries(Path zipPath, Charset charset) throws IOException {
        Map<String, String> text = new LinkedHashMap<>();
        for (Map.Entry<String, byte[]> entry : entries(zipPath).entrySet()) {
            if (isTextEntry(entry.getKey())) {
                text.put(entry.getKey(), new String(entry.getValue(), charset));
            }
        }
        return text;
    }

    private byte[] readEntry(ZipInputStream input) throws IOException {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        byte[] buffer = new byte[8192];
        int read;
        while ((read = input.read(buffer)) >= 0) {
            output.write(buffer, 0, read);
        }
        return output.toByteArray();
    }

    private boolean isTextEntry(String name) {
        String lower = name.toLowerCase();
        return lower.endsWith(".txt")
                || lower.endsWith(".ycc")
                || lower.endsWith(".java")
                || lower.endsWith(".csv")
                || lower.endsWith(".log");
    }
}
