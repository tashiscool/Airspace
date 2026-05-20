package org.tash.extensions.weather.decision;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class StormCellLifecycleTracker {
    public List<StormCellLifecycle> track(List<WeatherProduct> products) {
        if (products == null || products.isEmpty()) {
            return Collections.emptyList();
        }
        List<StormCellLifecycle> lifecycles = new ArrayList<>();
        for (WeatherProduct product : products) {
            StormCellLifecycle existing = nearest(lifecycles, product);
            if (existing == null) {
                List<WeatherProduct> values = new ArrayList<>();
                values.add(product);
                lifecycles.add(StormCellLifecycle.builder()
                        .lifecycleId("storm-" + lifecycles.size())
                        .state(state(product))
                        .products(values)
                        .build());
            } else {
                List<WeatherProduct> values = new ArrayList<>(existing.getProducts());
                values.add(product);
                lifecycles.set(lifecycles.indexOf(existing), StormCellLifecycle.builder()
                        .lifecycleId(existing.getLifecycleId())
                        .state(state(product))
                        .products(values)
                        .build());
            }
        }
        return lifecycles;
    }

    private StormCellLifecycle nearest(List<StormCellLifecycle> lifecycles, WeatherProduct product) {
        GeoCoordinate center = center(product);
        if (center == null) return null;
        for (StormCellLifecycle lifecycle : lifecycles) {
            WeatherProduct previous = lifecycle.getProducts().isEmpty() ? null : lifecycle.getProducts().get(lifecycle.getProducts().size() - 1);
            GeoCoordinate previousCenter = center(previous);
            if (previousCenter != null && previousCenter.distanceTo(center) <= 80.0) {
                return lifecycle;
            }
        }
        return null;
    }

    private GeoCoordinate center(WeatherProduct product) {
        if (product == null || product.getGeometry().isEmpty()) return null;
        double lat = 0.0;
        double lon = 0.0;
        for (GeoCoordinate point : product.getGeometry()) {
            lat += point.getLatitude();
            lon += point.getLongitude();
        }
        return GeoCoordinate.builder()
                .latitude(lat / product.getGeometry().size())
                .longitude(lon / product.getGeometry().size())
                .build();
    }

    private StormLifecycleState state(WeatherProduct product) {
        if (product == null) return StormLifecycleState.UNKNOWN;
        if (product.getStormPhase() != null) {
            try {
                return StormLifecycleState.valueOf(product.getStormPhase());
            } catch (IllegalArgumentException ignored) {
                // Fall through to trend.
            }
        }
        if (product.getGrowthTrend() != null && product.getGrowthTrend() > 0.0) return StormLifecycleState.GROWING;
        if (product.getGrowthTrend() != null && product.getGrowthTrend() < 0.0) return StormLifecycleState.DECAYING;
        return StormLifecycleState.UNKNOWN;
    }
}
