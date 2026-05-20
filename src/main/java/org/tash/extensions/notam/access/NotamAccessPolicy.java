package org.tash.extensions.notam.access;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class NotamAccessPolicy {
    private final NotamAccessReferenceData referenceData;

    public NotamAccessPolicy(NotamAccessReferenceData referenceData) {
        this.referenceData = referenceData;
    }

    public NotamAccessResult verify(NotamAccessRequest request) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        if (request == null) {
            errors.add("No NOTAM access request supplied");
            return result(false, warnings, errors);
        }
        String source = value(request.getSource());
        String privilege = value(request.getPrivilege());
        if (source.equals("D") && referenceData.hasAnyDomesticPrivilege(privilege)) {
            return result(true, warnings, errors);
        }
        if (source.equals("N") && referenceData.hasAnyInternationalPrivilege(privilege)) {
            return result(true, warnings, errors);
        }
        if ((source.equals("M") || source.equals("O")) && referenceData.hasAnyMilitaryPrivilege(privilege)) {
            return result(true, warnings, errors);
        }
        if (referenceData.hasAccountPrivilege(privilege, request.getAccountId(), request.getSeries(), source)) {
            if (allLocationsAllowed(request, privilege, source, errors)) {
                return result(true, warnings, errors);
            }
            return result(false, warnings, errors);
        }
        if (allLocationsAllowed(request, privilege, source, errors)) {
            return result(true, warnings, errors);
        }
        errors.add("Privilege " + privilege + " is not authorized for requested NOTAM account/source/locations");
        return result(false, warnings, errors);
    }

    private boolean allLocationsAllowed(NotamAccessRequest request,
                                        String privilege,
                                        String source,
                                        List<String> errors) {
        if (request.getLocationIds() == null || request.getLocationIds().isEmpty()) {
            return false;
        }
        for (String location : request.getLocationIds()) {
            if (!referenceData.hasLocationPrivilege(privilege, location, request.getSeries(), source)) {
                errors.add("Location " + value(location) + " is not authorized");
                return false;
            }
        }
        return true;
    }

    private NotamAccessResult result(boolean allowed, List<String> warnings, List<String> errors) {
        return NotamAccessResult.builder()
                .allowed(allowed)
                .warnings(Collections.unmodifiableList(new ArrayList<>(warnings)))
                .errors(Collections.unmodifiableList(new ArrayList<>(errors)))
                .build();
    }

    private String value(String value) {
        return value == null ? "" : value.trim().toUpperCase(Locale.US);
    }
}
