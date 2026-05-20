package org.tash.extensions.notam.access;

import java.util.Collections;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;

public class InMemoryNotamAccessReferenceData implements NotamAccessReferenceData {
    private final Set<String> accountPrivileges;
    private final Set<String> locationPrivileges;
    private final Set<String> anyPrivileges;

    public InMemoryNotamAccessReferenceData(Set<String> accountPrivileges,
                                            Set<String> locationPrivileges,
                                            Set<String> anyPrivileges) {
        this.accountPrivileges = normalize(accountPrivileges);
        this.locationPrivileges = normalize(locationPrivileges);
        this.anyPrivileges = normalize(anyPrivileges);
    }

    @Override
    public boolean hasAccountPrivilege(String privilege, String accountId, String series, String source) {
        return accountPrivileges.contains(key(privilege, accountId, series, source));
    }

    @Override
    public boolean hasLocationPrivilege(String privilege, String locationId, String series, String source) {
        return locationPrivileges.contains(key(privilege, locationId, series, source));
    }

    @Override
    public boolean hasAnyDomesticPrivilege(String privilege) {
        return anyPrivileges.contains(key(privilege, "DOM"));
    }

    @Override
    public boolean hasAnyInternationalPrivilege(String privilege) {
        return anyPrivileges.contains(key(privilege, "INT"));
    }

    @Override
    public boolean hasAnyMilitaryPrivilege(String privilege) {
        return anyPrivileges.contains(key(privilege, "MIL"));
    }

    public static String key(String... parts) {
        StringBuilder builder = new StringBuilder();
        for (String part : parts) {
            if (builder.length() > 0) {
                builder.append('|');
            }
            builder.append(part == null ? "" : part.trim().toUpperCase(Locale.US));
        }
        return builder.toString();
    }

    private Set<String> normalize(Set<String> values) {
        if (values == null) {
            return Collections.emptySet();
        }
        Set<String> normalized = new HashSet<>();
        for (String value : values) {
            normalized.add(value == null ? "" : value.trim().toUpperCase(Locale.US));
        }
        return Collections.unmodifiableSet(normalized);
    }
}
