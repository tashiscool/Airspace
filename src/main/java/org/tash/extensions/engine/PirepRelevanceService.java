package org.tash.extensions.engine;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.pirep.PirepReport;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Mission-scoped PIREP filtering: corridor proximity, route altitude tolerance,
 * and recent observation window. The default policy mirrors the FAA-gap target:
 * route corridor, altitude +/- 2000 ft, last 60 minutes.
 */
public class PirepRelevanceService {
    public PirepRelevanceResult filter(PirepRelevanceRequest request) {
        PirepRelevanceRequest safe = request == null ? PirepRelevanceRequest.builder().build() : request;
        List<String> diagnostics = new ArrayList<>();
        List<PirepReport> relevant = new ArrayList<>();
        ZonedDateTime decisionTime = safe.getDecisionTime() == null ? ZonedDateTime.now() : safe.getDecisionTime();
        Duration recency = safe.getRecencyWindow() == null ? Duration.ofMinutes(60) : safe.getRecencyWindow();
        double routeAltitude = safe.getRouteAltitudeFeet() == null ? averageAltitude(safe.getRoute()) : safe.getRouteAltitudeFeet();
        for (PirepReport report : safe.getPireps() == null ? Collections.<PirepReport>emptyList() : safe.getPireps()) {
            if (report == null) {
                continue;
            }
            if (report.getLocation() == null) {
                diagnostics.add(reportId(report) + " missing location");
                continue;
            }
            if (report.getObservationTime() == null) {
                diagnostics.add(reportId(report) + " missing observation time");
                continue;
            }
            long ageMinutes = Math.abs(Duration.between(report.getObservationTime(), decisionTime).toMinutes());
            if (ageMinutes > recency.toMinutes()) {
                diagnostics.add(reportId(report) + " outside recency window");
                continue;
            }
            if (report.getAltitudeFeet() != null && !Double.isNaN(routeAltitude)
                    && Math.abs(report.getAltitudeFeet() - routeAltitude) > safe.getAltitudeToleranceFeet()) {
                diagnostics.add(reportId(report) + " outside altitude tolerance");
                continue;
            }
            if (!safe.getRoute().isEmpty() && distanceToRoute(safe.getRoute(), report.getLocation()) > safe.getRouteBufferNauticalMiles()) {
                diagnostics.add(reportId(report) + " outside route corridor");
                continue;
            }
            relevant.add(report);
        }
        return PirepRelevanceResult.builder().relevant(relevant).diagnostics(diagnostics).build();
    }

    private double averageAltitude(List<GeoCoordinate> route) {
        if (route == null || route.isEmpty()) {
            return Double.NaN;
        }
        double total = 0.0;
        int count = 0;
        for (GeoCoordinate point : route) {
            total += point.getAltitude();
            count++;
        }
        return count == 0 ? Double.NaN : total / count;
    }

    private double distanceToRoute(List<GeoCoordinate> route, GeoCoordinate point) {
        double min = Double.POSITIVE_INFINITY;
        for (GeoCoordinate routePoint : route) {
            min = Math.min(min, routePoint.distanceTo(point));
        }
        return min;
    }

    private String reportId(PirepReport report) {
        return report.getId() == null ? "PIREP" : report.getId();
    }
}
