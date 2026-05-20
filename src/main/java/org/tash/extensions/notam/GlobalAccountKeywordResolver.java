package org.tash.extensions.notam;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

public class GlobalAccountKeywordResolver {
    private final Set<String> globalAccounts;

    public GlobalAccountKeywordResolver() {
        this(defaultAccounts());
    }

    public GlobalAccountKeywordResolver(Set<String> globalAccounts) {
        Set<String> normalized = new HashSet<>();
        for (String account : globalAccounts) {
            normalized.add(normalize(account));
        }
        this.globalAccounts = Collections.unmodifiableSet(normalized);
    }

    public boolean isGlobalAccount(String accountability) {
        String normalized = normalize(accountability);
        return globalAccounts.contains(normalized) || normalized.startsWith("NC");
    }

    public String defaultKeyword(String accountability) {
        return "GPS".equalsIgnoreCase(accountability) || "KGPS".equalsIgnoreCase(accountability)
                ? "NAV"
                : "AIRSPACE";
    }

    public Set<String> accounts() {
        return globalAccounts;
    }

    public static GlobalAccountKeywordResolver fromLegacySpreadsheet(Path path) throws IOException {
        Set<String> accounts = new HashSet<>(defaultAccounts());
        for (String text : printableStrings(path, 3)) {
            for (String token : text.split("[^A-Za-z0-9]+")) {
                String normalized = normalize(token);
                if (normalized.matches("K?[A-Z0-9]{3,4}") && isLikelyAccount(normalized)) {
                    accounts.add(normalized.startsWith("K") && normalized.length() == 4
                            ? normalized.substring(1)
                            : normalized);
                }
            }
        }
        return new GlobalAccountKeywordResolver(accounts);
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

    private static void addIfLongEnough(List<String> strings, StringBuilder current, int minimumLength) {
        if (current.length() >= minimumLength) {
            strings.add(current.toString());
        }
        current.setLength(0);
    }

    private static boolean isLikelyAccount(String token) {
        return token.equals("GPS") || token.equals("FDC") || token.equals("CARF")
                || token.equals("KCNF") || token.equals("KNSF") || token.equals("KQZC")
                || token.equals("KGPS") || token.equals("KFDC") || token.equals("USD")
                || token.equals("UAR") || token.equals("WAA") || token.equals("ETAXG");
    }

    private static Set<String> defaultAccounts() {
        Set<String> accounts = new HashSet<>();
        Collections.addAll(accounts, "GPS", "FDC", "DCC", "CARF", "NCR", "NCS",
                "CNF", "NSF", "QZC", "USD", "UAR", "WAA", "ETAXG");
        return accounts;
    }

    private static String normalize(String value) {
        return value == null ? "" : value.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }
}
