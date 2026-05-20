package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ReferenceDataImportRecord {
    private final String identifier;
    private final String pointType;
    private final double latitude;
    private final double longitude;
    private final Double altitudeFeet;
    private final String source;
    private final String sourceVersion;
    private final String metadataJson;
}
