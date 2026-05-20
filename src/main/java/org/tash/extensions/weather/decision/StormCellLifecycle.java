package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class StormCellLifecycle {
    private final String lifecycleId;
    private final StormLifecycleState state;
    @Builder.Default
    private final List<WeatherProduct> products = new ArrayList<>();

    public List<WeatherProduct> getProducts() {
        return Collections.unmodifiableList(products == null ? Collections.emptyList() : products);
    }
}
