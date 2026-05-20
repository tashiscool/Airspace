package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.CarfWaypointCandidate;
import org.tash.extensions.reservation.CompositeCarfWaypointResolver;
import org.tash.extensions.reservation.MapWaypointResolver;
import org.tash.extensions.reservation.PrioritizedCarfWaypointResolver;
import org.tash.extensions.reservation.WaypointFileResolver;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class CarfWaypointResolverTest {
    @Test
    void duplicateNavaidsPreferConfiguredCandidate() {
        PrioritizedCarfWaypointResolver resolver = new PrioritizedCarfWaypointResolver(Arrays.asList(
                candidate("ABC", "KZ", 10, -10, false),
                candidate("ABC", "PA", 20, -20, true),
                candidate("ABC", "PH", 30, -30, false)));

        assertEquals(20, resolver.resolve("abc").orElseThrow(AssertionError::new).getLatitude(), 0.0001);
        assertEquals("PA", resolver.resolveCandidate("ABC").orElseThrow(AssertionError::new).getCountryCode());
    }

    @Test
    void duplicateNavaidsPreferUsAndFaaRegionsBeforeForeignFallback() {
        PrioritizedCarfWaypointResolver resolver = new PrioritizedCarfWaypointResolver(Arrays.asList(
                candidate("DUP", "EG", 10, -10, false),
                candidate("DUP", "TJ", 50, -50, false),
                candidate("DUP", "PH", 30, -30, false),
                candidate("DUP", "K7", 20, -20, false),
                candidate("DUP", "PA", 40, -40, false)));

        assertEquals("K7", resolver.resolveCandidate("DUP").orElseThrow(AssertionError::new).getCountryCode());
        assertEquals(5, resolver.candidates("DUP").size());
    }

    @Test
    void duplicateNavaidsFallBackToFirstForeignCandidateWhenNoPreferredRegionExists() {
        PrioritizedCarfWaypointResolver resolver = new PrioritizedCarfWaypointResolver(Arrays.asList(
                candidate("INTL", "EG", 10, -10, false),
                candidate("INTL", "CY", 20, -20, false)));

        assertEquals("EG", resolver.resolveCandidate("INTL").orElseThrow(AssertionError::new).getCountryCode());
    }

    @Test
    void compositeResolverAllowsExternalSourcesAheadOfFixtureCatalogs() {
        MapWaypointResolver externalKvmLikeSource = new MapWaypointResolver(Collections.singletonMap(
                "KVMFIX",
                GeoCoordinate.builder().latitude(44.0).longitude(-70.0).altitude(0).build()));
        MapWaypointResolver fixtureFallback = new MapWaypointResolver(Collections.singletonMap(
                "KVMFIX",
                GeoCoordinate.builder().latitude(10.0).longitude(-20.0).altitude(0).build()));

        CompositeCarfWaypointResolver resolver =
                new CompositeCarfWaypointResolver(externalKvmLikeSource, fixtureFallback);

        assertEquals(44.0, resolver.resolve("KVMFIX").orElseThrow(AssertionError::new).getLatitude(), 0.0001);
        assertEquals(2, resolver.resolvers().size());
    }

    @Test
    void waypointFileResolverLoadsExternalDelimitedExports() throws Exception {
        Path export = Files.createTempFile("kvm-waypoints", ".csv");
        Files.write(export, Arrays.asList(
                "identifier,latitude,longitude,altitude",
                "KVMFIX,44.1,-70.2,1200",
                "NASR1 35.5 -101.2",
                "# comments are ignored"));

        WaypointFileResolver resolver = WaypointFileResolver.fromDelimited(export);

        assertEquals(44.1, resolver.resolve("KVMFIX").orElseThrow(AssertionError::new).getLatitude(), 0.0001);
        assertEquals(-70.2, resolver.resolve("KVMFIX").orElseThrow(AssertionError::new).getLongitude(), 0.0001);
        assertEquals(1200, resolver.resolve("KVMFIX").orElseThrow(AssertionError::new).getAltitude(), 0.0001);
        assertEquals(35.5, resolver.resolve("NASR1").orElseThrow(AssertionError::new).getLatitude(), 0.0001);
    }

    private CarfWaypointCandidate candidate(String identifier,
                                            String countryCode,
                                            double latitude,
                                            double longitude,
                                            boolean preferred) {
        return CarfWaypointCandidate.builder()
                .identifier(identifier)
                .countryCode(countryCode)
                .preferred(preferred)
                .coordinate(GeoCoordinate.builder()
                        .latitude(latitude)
                        .longitude(longitude)
                        .altitude(0)
                        .build())
                .build();
    }
}
