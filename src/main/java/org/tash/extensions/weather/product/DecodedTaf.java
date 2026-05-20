package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecodedTaf {
    private final String stationId;
    private final boolean amended;
    private final boolean corrected;
    @Builder.Default
    private final List<DecodedForecastGroup> groups = new ArrayList<>();

    public List<DecodedForecastGroup> getGroups() {
        return Collections.unmodifiableList(groups == null ? Collections.emptyList() : groups);
    }
}
