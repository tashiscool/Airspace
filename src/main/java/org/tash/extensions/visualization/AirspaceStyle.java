package org.tash.extensions.visualization;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class AirspaceStyle {
    private String stroke;
    private String fill;
    private Double strokeWidth;
    private Double fillOpacity;
}
