package org.tash.extensions.weather.product;

import java.util.ArrayList;
import java.util.List;

public class TafDecoder extends AbstractWeatherProductDecoder {
    public TafDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.TAF);
    }

    @Override
    protected Object decoded(WeatherProductParseResult result) {
        WeatherProduct product = result == null ? null : result.getProduct();
        if (product == null) {
            return null;
        }
        List<DecodedForecastGroup> groups = new ArrayList<>();
        for (WeatherForecastSlice slice : product.getForecastSlices()) {
            groups.add(DecodedForecastGroup.builder()
                    .groupType(slice.getGroupType())
                    .rawText(slice.getRawText())
                    .validStart(slice.getValidStart())
                    .validEnd(slice.getValidEnd())
                    .confidence(slice.getConfidence())
                    .ceilingFeet(slice.getCeilingFeet())
                    .visibilityStatuteMiles(slice.getVisibilityStatuteMiles())
                    .build());
        }
        return DecodedTaf.builder()
                .stationId(product.getStationId())
                .amended(Boolean.TRUE.equals(product.getAmended()))
                .corrected(Boolean.TRUE.equals(product.getCorrected()))
                .groups(groups)
                .build();
    }
}
