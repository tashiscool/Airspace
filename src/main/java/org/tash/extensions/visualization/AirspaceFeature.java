package org.tash.extensions.visualization;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.LinkedHashMap;
import java.util.Map;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class AirspaceFeature {
    @Builder.Default
    private String type = "Feature";
    private String id;
    private AirspaceGeometry geometry;
    @Builder.Default
    private Map<String, Object> properties = new LinkedHashMap<>();
}
