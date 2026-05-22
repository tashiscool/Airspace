package org.tash.extensions.product.application;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.Collections;
import java.util.List;

/**
 * Deterministic cost-estimation seam for route comparison. Values are explicit
 * defaults for prototype planning and can be replaced by airline/operator
 * economics without changing product DTOs.
 */
public class OperationalCostModel {
    private static final double DEFAULT_CRUISE_SPEED_KNOTS = 450.0;
    private static final double DEFAULT_FUEL_BURN_LB_PER_NM = 15.0;
    private static final double DEFAULT_FUEL_COST_USD_PER_LB = 0.95;
    private static final double DEFAULT_DELAY_COST_USD_PER_MINUTE = 75.0;

    private final double cruiseSpeedKnots;
    private final double fuelBurnLbPerNm;
    private final double fuelCostUsdPerLb;
    private final double delayCostUsdPerMinute;

    public OperationalCostModel() {
        this(DEFAULT_CRUISE_SPEED_KNOTS, DEFAULT_FUEL_BURN_LB_PER_NM,
                DEFAULT_FUEL_COST_USD_PER_LB, DEFAULT_DELAY_COST_USD_PER_MINUTE);
    }

    public OperationalCostModel(double cruiseSpeedKnots,
                                double fuelBurnLbPerNm,
                                double fuelCostUsdPerLb,
                                double delayCostUsdPerMinute) {
        this.cruiseSpeedKnots = cruiseSpeedKnots <= 0 ? DEFAULT_CRUISE_SPEED_KNOTS : cruiseSpeedKnots;
        this.fuelBurnLbPerNm = fuelBurnLbPerNm < 0 ? DEFAULT_FUEL_BURN_LB_PER_NM : fuelBurnLbPerNm;
        this.fuelCostUsdPerLb = fuelCostUsdPerLb < 0 ? DEFAULT_FUEL_COST_USD_PER_LB : fuelCostUsdPerLb;
        this.delayCostUsdPerMinute = delayCostUsdPerMinute < 0 ? DEFAULT_DELAY_COST_USD_PER_MINUTE : delayCostUsdPerMinute;
    }

    public RouteCostEstimate estimateOriginal(List<GeoCoordinate> originalRoute) {
        double distance = distance(originalRoute);
        return estimate(distance, distance);
    }

    public RouteCostEstimate estimate(List<GeoCoordinate> originalRoute, List<GeoCoordinate> candidateRoute) {
        return estimate(distance(originalRoute), distance(candidateRoute));
    }

    private RouteCostEstimate estimate(double originalDistance, double candidateDistance) {
        double additionalDistance = Math.max(0.0, candidateDistance - originalDistance);
        double minutes = candidateDistance / cruiseSpeedKnots * 60.0;
        double originalMinutes = originalDistance / cruiseSpeedKnots * 60.0;
        double fuel = candidateDistance * fuelBurnLbPerNm;
        double originalFuel = originalDistance * fuelBurnLbPerNm;
        double fuelCost = fuel * fuelCostUsdPerLb;
        double originalFuelCost = originalFuel * fuelCostUsdPerLb;
        double additionalMinutes = Math.max(0.0, minutes - originalMinutes);
        double estimatedCost = fuelCost + additionalMinutes * delayCostUsdPerMinute;
        double originalCost = originalFuelCost;
        return RouteCostEstimate.builder()
                .distanceNm(candidateDistance)
                .additionalDistanceNm(additionalDistance)
                .estimatedMinutes(minutes)
                .additionalMinutes(additionalMinutes)
                .estimatedFuelLb(fuel)
                .additionalFuelLb(Math.max(0.0, fuel - originalFuel))
                .estimatedCostUsd(estimatedCost)
                .additionalCostUsd(Math.max(0.0, estimatedCost - originalCost))
                .cruiseSpeedKnots(cruiseSpeedKnots)
                .fuelBurnLbPerNm(fuelBurnLbPerNm)
                .fuelCostUsdPerLb(fuelCostUsdPerLb)
                .delayCostUsdPerMinute(delayCostUsdPerMinute)
                .build();
    }

    private double distance(List<GeoCoordinate> route) {
        List<GeoCoordinate> safe = route == null ? Collections.emptyList() : route;
        double total = 0.0;
        for (int i = 0; i + 1 < safe.size(); i++) {
            total += safe.get(i).distanceTo(safe.get(i + 1));
        }
        return total;
    }

    @Data
    @Builder
    public static class RouteCostEstimate {
        private double distanceNm;
        private double additionalDistanceNm;
        private double estimatedMinutes;
        private double additionalMinutes;
        private double estimatedFuelLb;
        private double additionalFuelLb;
        private double estimatedCostUsd;
        private double additionalCostUsd;
        private double cruiseSpeedKnots;
        private double fuelBurnLbPerNm;
        private double fuelCostUsdPerLb;
        private double delayCostUsdPerMinute;
    }
}
