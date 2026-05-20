package org.tash.extensions.weather.product;

import java.util.ArrayList;
import java.util.List;

public class MetarDecoder extends AbstractWeatherProductDecoder {
    public MetarDecoder(WeatherProductParser parser) {
        super(parser, WeatherProductType.METAR);
    }

    @Override
    protected Object decoded(WeatherProductParseResult result) {
        WeatherProduct product = result == null ? null : result.getProduct();
        if (product == null) {
            return null;
        }
        List<DecodedCloudLayer> cloudLayers = new ArrayList<>();
        for (String layer : product.getCloudLayers()) {
            String coverage = layer.length() >= 3 ? layer.substring(0, 3) : layer;
            Double base = layer.length() >= 6 ? Double.parseDouble(layer.substring(3, 6)) * 100.0 : null;
            cloudLayers.add(DecodedCloudLayer.builder()
                    .coverage(coverage)
                    .baseFeet(base)
                    .cumulonimbus(layer.contains("CB"))
                    .build());
        }
        return DecodedMetar.builder()
                .stationId(product.getStationId())
                .reportTime(product.getIssuedAt())
                .corrected(Boolean.TRUE.equals(product.getCorrected()))
                .windDirectionDegrees(product.getWindDirectionDegrees())
                .windSpeedKnots(product.getWindSpeedKnots())
                .windGustKnots(product.getWindGustKnots())
                .variableWindFromDegrees(product.getVariableWindFromDegrees())
                .variableWindToDegrees(product.getVariableWindToDegrees())
                .visibilityStatuteMiles(product.getVisibilityStatuteMiles())
                .runwayVisualRangeFeet(product.getRunwayVisualRangeFeet())
                .temperatureCelsius(product.getTemperatureCelsius())
                .dewpointCelsius(product.getDewpointCelsius())
                .altimeterInchesHg(product.getAltimeterInchesHg())
                .remarks(product.getRemarks())
                .presentWeather(new ArrayList<>(product.getWeatherPhenomena()))
                .cloudLayers(cloudLayers)
                .build();
    }
}
