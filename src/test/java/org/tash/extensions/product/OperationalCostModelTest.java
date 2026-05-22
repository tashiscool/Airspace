package org.tash.extensions.product;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.product.application.OperationalCostModel;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class OperationalCostModelTest {
    @Test
    void estimatesDistanceDelayFuelAndCostDeltasDeterministically() {
        OperationalCostModel model = new OperationalCostModel(300.0, 10.0, 1.0, 50.0);
        List<GeoCoordinate> original = Arrays.asList(point(30.0, -150.0), point(30.0, -149.0));
        List<GeoCoordinate> candidate = Arrays.asList(point(30.0, -150.0), point(31.0, -149.5), point(30.0, -149.0));

        OperationalCostModel.RouteCostEstimate estimate = model.estimate(original, candidate);

        assertTrue(estimate.getDistanceNm() > 0.0);
        assertTrue(estimate.getAdditionalDistanceNm() > 0.0);
        assertTrue(estimate.getAdditionalMinutes() > 0.0);
        assertEquals(estimate.getAdditionalDistanceNm() * 10.0, estimate.getAdditionalFuelLb(), 0.0001);
        assertTrue(estimate.getAdditionalCostUsd() > estimate.getAdditionalFuelLb());
    }

    private GeoCoordinate point(double lat, double lon) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(24000).build();
    }
}
