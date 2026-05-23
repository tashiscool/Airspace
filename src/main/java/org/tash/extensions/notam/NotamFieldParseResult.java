package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
public class NotamFieldParseResult {
    private boolean accepted;
    private String notamType;
    private String qField;
    private String aField;
    private String bField;
    private String cField;
    private String dField;
    private String eField;
    private String fField;
    private String gField;
    private String accountability;
    private String qCode;
    private String traffic;
    private String purpose;
    private String scope;
    private String lowerFlightLevel;
    private String upperFlightLevel;
    private boolean estimatedEnd;
    private boolean permanentEnd;
    private boolean hasGeometry;
    private Double centerLatitude;
    private Double centerLongitude;
    private Double radiusNauticalMiles;
    @Builder.Default
    private List<String> warnings = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
    private String rawText;
}
