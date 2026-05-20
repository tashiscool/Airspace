package org.tash.extensions.repository;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;

import java.util.Optional;

public interface ReferenceDataRepository extends CarfReferenceDataProvider {
    default Optional<GeoCoordinate> findFixOrNavaid(String id) {
        return resolveFixOrNavaid(id);
    }

    default Optional<String> findAccount(String id) {
        return Optional.empty();
    }

    default Optional<String> findGlobalAccount(String id) {
        return Optional.empty();
    }
}
