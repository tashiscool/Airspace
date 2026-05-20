package org.tash.extensions.carf.refdata;

import org.tash.data.GeoCoordinate;

import java.util.Optional;

public interface CarfReferenceDataProvider {
    Optional<GeoCoordinate> resolveFixOrNavaid(String id);

    default Optional<GeoCoordinate> resolvePreferredNavaid(String id) {
        return Optional.empty();
    }

    default Optional<String> resolveAccountKeywordDefault(String accountId) {
        return Optional.empty();
    }

    default Optional<String> resolveGlobalAccount(String accountId) {
        return Optional.empty();
    }

    default Optional<String> resolveRouteReference(String routeId) {
        return Optional.empty();
    }
}
