package org.tash.extensions.notam;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Pattern;

public class FirReferenceCatalog {
    private static final Pattern IDENTIFIER = Pattern.compile("[A-Z0-9]{3,4}");
    private static final Pattern DESCRIPTION = Pattern.compile(
            ".*\\b(FIR|INTL|INTERNATIONAL|REGIONAL|CENTER|COUNTY|AIRPORT|FIELD|AFB|STATION)\\b.*");

    private final Set<String> identifiers;
    private final Set<String> descriptions;

    public FirReferenceCatalog(Set<String> identifiers, Set<String> descriptions) {
        this.identifiers = new LinkedHashSet<>(identifiers);
        this.descriptions = new LinkedHashSet<>(descriptions);
    }

    public boolean containsIdentifier(String identifier) {
        if (identifier == null) {
            return false;
        }
        return identifiers.contains(normalizeIdentifier(identifier));
    }

    public Set<String> identifiers() {
        return Collections.unmodifiableSet(identifiers);
    }

    public Set<String> descriptions() {
        return Collections.unmodifiableSet(descriptions);
    }

    public static FirReferenceCatalog fromLegacySpreadsheet(Path path) throws IOException {
        List<String> strings = new ArrayList<>();
        strings.addAll(printableStrings(path, 3));
        strings.addAll(printableUtf16LeStrings(path, 3));

        Set<String> identifiers = new LinkedHashSet<>();
        Set<String> descriptions = new LinkedHashSet<>();
        for (String text : strings) {
            for (String rawLine : text.replace('\r', '\n').split("\\n")) {
                String line = rawLine.trim().replaceAll("\\s+", " ");
                if (line.isEmpty()) {
                    continue;
                }
                String identifier = normalizeIdentifier(line);
                if (IDENTIFIER.matcher(identifier).matches()) {
                    identifiers.add(identifier);
                }
                String upper = line.toUpperCase(Locale.US);
                if (DESCRIPTION.matcher(upper).matches()) {
                    descriptions.add(line);
                }
            }
        }
        return new FirReferenceCatalog(identifiers, descriptions);
    }

    private static List<String> printableStrings(Path path, int minimumLength) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        List<String> strings = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        for (byte value : bytes) {
            int ch = value & 0xff;
            if ((ch >= 32 && ch <= 126) || ch == '\n' || ch == '\r' || ch == '\t') {
                current.append((char) ch);
            } else {
                addIfLongEnough(strings, current, minimumLength);
            }
        }
        addIfLongEnough(strings, current, minimumLength);
        return strings;
    }

    private static List<String> printableUtf16LeStrings(Path path, int minimumLength) throws IOException {
        byte[] bytes = Files.readAllBytes(path);
        List<String> strings = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        for (int i = 0; i + 1 < bytes.length; i += 2) {
            int ch = (bytes[i] & 0xff) | ((bytes[i + 1] & 0xff) << 8);
            if ((ch >= 32 && ch <= 126) || ch == '\n' || ch == '\r' || ch == '\t') {
                current.append((char) ch);
            } else {
                addIfLongEnough(strings, current, minimumLength);
            }
        }
        addIfLongEnough(strings, current, minimumLength);
        return strings;
    }

    private static void addIfLongEnough(List<String> strings, StringBuilder current, int minimumLength) {
        if (current.length() >= minimumLength) {
            strings.add(current.toString());
        }
        current.setLength(0);
    }

    private static String normalizeIdentifier(String identifier) {
        return identifier.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }
}
