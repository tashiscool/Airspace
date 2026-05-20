package org.tash.extensions.evaluation;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

public class LegacyArtifactTextExtractor {
    private static final Pattern NOTAM_START = Pattern.compile("^![A-Z0-9]{3,4}\\s+.*");

    public List<String> printableStrings(Path path, int minimumLength) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        List<String> strings = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        for (byte value : bytes) {
            int c = value & 0xff;
            if ((c >= 32 && c <= 126) || c == '\n' || c == '\r' || c == '\t') {
                current.append((char) c);
            } else {
                flush(strings, current, minimumLength);
            }
        }
        flush(strings, current, minimumLength);
        return strings;
    }

    public List<String> printableUtf16LeStrings(Path path, int minimumLength) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        String text = new String(bytes, StandardCharsets.UTF_16LE);
        List<String> strings = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if ((c >= 32 && c <= 126) || c == '\n' || c == '\r' || c == '\t') {
                current.append(c);
            } else {
                flush(strings, current, minimumLength);
            }
        }
        flush(strings, current, minimumLength);
        return strings;
    }

    public List<String> notamLines(Path path) throws IOException {
        Set<String> notams = new LinkedHashSet<>();
        List<String> extracted = new ArrayList<>();
        extracted.addAll(printableStrings(path, 8));
        extracted.addAll(printableUtf16LeStrings(path, 8));
        for (String text : extracted) {
            for (String line : text.replace('\r', '\n').split("\\n")) {
                String trimmed = line.trim().replaceAll("\\s+", " ");
                if (NOTAM_START.matcher(trimmed).matches()) {
                    notams.add(trimmed);
                }
            }
        }
        return new ArrayList<>(notams);
    }

    public boolean containsToken(Path path, String token) throws IOException {
        String wanted = token.toUpperCase();
        for (String text : printableStrings(path, Math.max(3, token.length()))) {
            if (text.toUpperCase().contains(wanted)) {
                return true;
            }
        }
        return false;
    }

    private void flush(List<String> strings, StringBuilder current, int minimumLength) {
        String value = current.toString().trim();
        if (value.length() >= minimumLength) {
            strings.add(value);
        }
        current.setLength(0);
    }
}
