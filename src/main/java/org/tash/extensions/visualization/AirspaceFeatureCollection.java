package org.tash.extensions.visualization;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class AirspaceFeatureCollection {
    @Builder.Default
    private String type = "FeatureCollection";
    @Builder.Default
    private List<AirspaceFeature> features = new ArrayList<>();
}
