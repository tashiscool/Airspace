package org.tash.extensions.carf.refdata;

import org.tash.data.GeoCoordinate;

import java.util.Collections;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

public class InMemoryCarfReferenceDataProvider implements CarfReferenceDataProvider {
    private final Map<String, GeoCoordinate> fixes;
    private final Map<String, String> accountDefaults;

    public InMemoryCarfReferenceDataProvider(Map<String, GeoCoordinate> fixes) {
        this(fixes, Collections.emptyMap());
    }

    public InMemoryCarfReferenceDataProvider(Map<String, GeoCoordinate> fixes, Map<String, String> accountDefaults) {
        Map<String, GeoCoordinate> normalized = new HashMap<>();
        for (Map.Entry<String, GeoCoordinate> entry : fixes.entrySet()) {
            normalized.put(normalize(entry.getKey()), entry.getValue());
        }
        this.fixes = Collections.unmodifiableMap(normalized);
        Map<String, String> defaults = new HashMap<>();
        for (Map.Entry<String, String> entry : accountDefaults.entrySet()) {
            defaults.put(normalize(entry.getKey()), entry.getValue());
        }
        this.accountDefaults = Collections.unmodifiableMap(defaults);
    }

    @Override
    public Optional<GeoCoordinate> resolveFixOrNavaid(String id) {
        return Optional.ofNullable(fixes.get(normalize(id)));
    }

    @Override
    public Optional<String> resolveAccountKeywordDefault(String accountId) {
        return Optional.ofNullable(accountDefaults.get(normalize(accountId)));
    }

    @Override
    public Optional<String> resolveGlobalAccount(String accountId) {
        return Optional.ofNullable(accountDefaults.get(normalize(accountId)));
    }

    private String normalize(String value) {
        return value == null ? "" : value.toUpperCase(Locale.US).replaceAll("[^A-Z0-9]", "");
    }
}
