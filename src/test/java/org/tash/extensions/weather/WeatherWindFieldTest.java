package org.tash.extensions.weather;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.wind.AnalyticalWindField;
import org.tash.extensions.weather.wind.GriddedWindField;
import org.tash.extensions.weather.wind.InterpolatedWindField;
import org.tash.extensions.weather.wind.WindFieldModel;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WeatherWindFieldTest {
    private final ZonedDateTime baseTime = ZonedDateTime.parse("2026-05-20T12:00:00Z");

    @Test
    void analyticalConstantGradientAndThermalWindFieldsReturnExpectedComponents() {
        GeoCoordinate location = GeoCoordinate.builder().latitude(40.0).longitude(-75.0).altitude(12000).build();
        AnalyticalWindField constant = AnalyticalWindField.constantWind("constant", baseTime, 270, 40);

        assertEquals("constant", constant.getId());
        assertEquals(WeatherElementType.WIND, constant.getType());
        assertTrue(constant.isValidAt(baseTime.plusMinutes(59)));
        assertFalse(constant.isValidAt(baseTime.plusMinutes(61)));
        assertEquals(270.0, constant.getWindAt(location, baseTime).getDirection(), 0.0001);
        assertEquals(40.0, constant.getWindAt(location, baseTime).getSpeed(), 0.0001);

        AnalyticalWindField gradient = AnalyticalWindField.verticalGradientWind(
                "gradient", baseTime, 180, 20, 2, 1.5);
        WindComponent gradientWind = gradient.getWindAt(location, baseTime);
        assertEquals(204.0, gradientWind.getDirection(), 0.0001);
        assertEquals(38.0, gradientWind.getSpeed(), 0.0001);

        AnalyticalWindField thermal = AnalyticalWindField.withThermal(
                "thermal", baseTime, constant, location, 20.0, 12.0);
        assertEquals(12.0, thermal.getWindAt(location, baseTime).getVerticalSpeed(), 0.0001);
    }

    @Test
    void griddedWindFieldInterpolatesAndReturnsCalmOutsideBounds() {
        GriddedWindField field = GriddedWindField.builder()
                .id("grid")
                .validityTime(baseTime)
                .gridResolution(1.0)
                .altitudeLevels(Collections.singletonList(10000.0))
                .minLat(0.0)
                .maxLat(1.0)
                .minLon(10.0)
                .maxLon(11.0)
                .build();
        field.setWindData(0, 0, 0, wind(0, 10));
        field.setWindData(0, 1, 0, wind(90, 10));
        field.setWindData(1, 0, 0, wind(180, 10));
        field.setWindData(1, 1, 0, wind(270, 10));

        WindComponent interpolated = field.getWindAt(
                GeoCoordinate.builder().latitude(0.5).longitude(10.5).altitude(10000).build(),
                baseTime);
        assertEquals(0.0, interpolated.getSpeed(), 0.0001);

        WindComponent outside = field.getWindAt(
                GeoCoordinate.builder().latitude(5.0).longitude(10.5).altitude(10000).build(),
                baseTime);
        assertEquals(0.0, outside.getSpeed(), 0.0001);
        assertEquals(0.0, outside.getVerticalSpeed(), 0.0001);
    }

    @Test
    void interpolatedWindFieldUsesBracketingTimesAndFallbacks() {
        WindFieldModel first = AnalyticalWindField.constantWind("first", baseTime, 0, 20);
        WindFieldModel second = AnalyticalWindField.constantWind("second", baseTime.plusHours(2), 90, 20);
        InterpolatedWindField field = new InterpolatedWindField("interpolated", baseTime, Arrays.asList(first, second));

        assertTrue(field.isValidAt(baseTime.plusHours(1)));
        assertFalse(field.isValidAt(baseTime.minusMinutes(1)));
        assertEquals(0.0, field.getWindAt(point(), baseTime).getDirection(), 0.0001);
        assertEquals(90.0, field.getWindAt(point(), baseTime.plusHours(2)).getDirection(), 0.0001);

        WindComponent midpoint = field.getWindAt(point(), baseTime.plusHours(1));
        assertEquals(45.0, midpoint.getDirection(), 0.0001);
        assertEquals(Math.sqrt(200.0), midpoint.getSpeed(), 0.0001);

        InterpolatedWindField empty = new InterpolatedWindField("empty", baseTime, Collections.emptyList());
        assertFalse(empty.isValidAt(baseTime));
        assertEquals(0.0, empty.getWindAt(point(), baseTime).getSpeed(), 0.0001);
    }

    private WindComponent wind(double direction, double speed) {
        return WindComponent.builder().direction(direction).speed(speed).verticalSpeed(0).build();
    }

    private GeoCoordinate point() {
        return GeoCoordinate.builder().latitude(40.0).longitude(-75.0).altitude(10000).build();
    }
}
