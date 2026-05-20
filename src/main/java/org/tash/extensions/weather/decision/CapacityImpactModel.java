package org.tash.extensions.weather.decision;

import org.tash.extensions.engine.OperationalGeometryService;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class CapacityImpactModel {
    private final OperationalGeometryService geometryService;

    public CapacityImpactModel() {
        this(new OperationalGeometryService());
    }

    public CapacityImpactModel(OperationalGeometryService geometryService) {
        this.geometryService = geometryService == null ? new OperationalGeometryService() : geometryService;
    }

    public List<SectorCapacityImpact> impacts(RouteBlockagePrediction prediction,
                                              List<SectorDemandSnapshot> sectorDemand,
                                              double confidence) {
        if (prediction == null || sectorDemand == null || sectorDemand.isEmpty() || prediction.getIntersections().isEmpty()) {
            return Collections.emptyList();
        }
        List<SectorCapacityImpact> impacts = new ArrayList<>();
        for (SectorDemandSnapshot demand : sectorDemand) {
            if (demand == null || demand.getSector() == null || demand.getSector().getBaselineCapacityPerHour() <= 0.0) {
                continue;
            }
            boolean overlaps = prediction.getIntersections().stream()
                    .anyMatch(intersection -> geometryService.overlaps(demand.getSector().getBoundary(),
                            java.util.Arrays.asList(intersection.getEstimatedEntryPoint(), intersection.getEstimatedExitPoint())));
            if (!overlaps) {
                continue;
            }
            double ratio = demand.getActiveDemandPerHour() / demand.getSector().getBaselineCapacityPerHour();
            double impact = clamp(prediction.getBlockedProbability() * confidence * Math.max(0.25, ratio));
            impacts.add(SectorCapacityImpact.builder()
                    .sectorId(demand.getSector().getId())
                    .demandCapacityRatio(ratio)
                    .capacityImpact(impact)
                    .rationale("Weather blockage intersects sector " + demand.getSector().getId()
                            + " at demand/capacity ratio " + ratio)
                    .build());
        }
        return impacts;
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
