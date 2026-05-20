package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class DecodedCloudLayer {
    private final String coverage;
    private final Double baseFeet;
    private final boolean cumulonimbus;
}
