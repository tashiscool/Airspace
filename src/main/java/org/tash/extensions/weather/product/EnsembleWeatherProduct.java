package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class EnsembleWeatherProduct {
    private final String id;
    @Builder.Default
    private final List<WeatherEnsembleMember> members = new ArrayList<>();

    public List<WeatherEnsembleMember> getMembers() {
        return Collections.unmodifiableList(members == null ? Collections.emptyList() : members);
    }

    public List<WeatherProduct> products() {
        List<WeatherProduct> products = new ArrayList<>();
        for (WeatherEnsembleMember member : getMembers()) {
            if (member != null && member.getProduct() != null) {
                products.add(member.getProduct());
            }
        }
        return products;
    }
}
