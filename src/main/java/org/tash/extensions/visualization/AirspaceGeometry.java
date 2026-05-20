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
public class AirspaceGeometry {
    private String type;
    @Builder.Default
    private List<?> coordinates = new ArrayList<>();
}
