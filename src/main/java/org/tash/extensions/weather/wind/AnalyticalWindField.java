package org.tash.extensions.weather.wind;

import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.WeatherElement;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.WindComponent;

import java.time.ZonedDateTime;
import java.util.function.Function;

/**
 * A wind field model that uses a mathematical function to generate wind data
 */
@Data
public class AnalyticalWindField extends AbstractWindField {
    /**
     * Function that generates wind based on location and time
     */
    private final Function<GeoCoordinate, WindComponent> windFunction;

    /**
     * Create a new analytical wind field
     *
     * @param id           Wind field identifier
     * @param validityTime Validity time
     * @param windFunction Function to generate wind data
     */
    public AnalyticalWindField(
            String id,
            ZonedDateTime validityTime,
            Function<GeoCoordinate, WindComponent> windFunction) {
        super(id, validityTime);
        this.windFunction = windFunction;
    }

    @Override
    public WindComponent getWindAt(GeoCoordinate location, ZonedDateTime time) {
        return windFunction.apply(location);
    }

    /**
     * Create a constant wind field with the same wind everywhere
     *
     * @param id           Wind field identifier
     * @param validityTime Validity time
     * @param direction    Wind direction (degrees)
     * @param speed        Wind speed (knots)
     * @return Constant wind field
     */
    public static AnalyticalWindField constantWind(
            String id, ZonedDateTime validityTime, double direction, double speed) {
        WindComponent wind = WindComponent.builder()
                .direction(direction)
                .speed(speed)
                .verticalSpeed(0)
                .build();

        return new AnalyticalWindField(
                id, validityTime, location -> wind);
    }

    /**
     * Create a wind field with vertical gradient (changing with altitude)
     *
     * @param id                Wind field identifier
     * @param validityTime      Validity time
     * @param surfaceDirection  Surface wind direction (degrees)
     * @param surfaceSpeed      Surface wind speed (knots)
     * @param directionGradient Direction change per 1000 feet (degrees)
     * @param speedGradient     Speed change per 1000 feet (knots)
     * @return Wind field with vertical gradient
     */
    public static AnalyticalWindField verticalGradientWind(
            String id, ZonedDateTime validityTime,
            double surfaceDirection, double surfaceSpeed,
            double directionGradient, double speedGradient) {

        return new AnalyticalWindField(
                id, validityTime,
                location -> {
                    double altitudeFactor = location.getAltitude() / 1000.0;
                    double direction = (surfaceDirection + directionGradient * altitudeFactor) % 360;
                    double speed = Math.max(0, surfaceSpeed + speedGradient * altitudeFactor);

                    return WindComponent.builder()
                            .direction(direction)
                            .speed(speed)
                            .verticalSpeed(0)
                            .build();
                });
    }

    /**
     * Create a wind field with a thermal (updraft)
     *
     * @param id           Wind field identifier
     * @param validityTime Validity time
     * @param baseWind     Base wind field
     * @param center       Thermal center location
     * @param radius       Thermal radius (nautical miles)
     * @param strength     Maximum vertical speed (knots)
     * @return Wind field with thermal
     */
    public static AnalyticalWindField withThermal(
            String id, ZonedDateTime validityTime,
            WindFieldModel baseWind,
            GeoCoordinate center, double radius, double strength) {

        return new AnalyticalWindField(
                id, validityTime,
                location -> {
                    // Calculate distance to thermal center
                    double distance = center.distanceTo(location);

                    // Get base wind
                    WindComponent baseWindComp = baseWind.getWindAt(location, validityTime);

                    // Calculate vertical wind component
                    double verticalSpeed = 0;
                    if (distance < radius) {
                        // Parabolic profile with maximum at center
                        double normalizedDist = distance / radius;
                        verticalSpeed = strength * (1 - normalizedDist * normalizedDist);
                    }

                    // Add vertical component to base wind
                    return WindComponent.builder()
                            .direction(baseWindComp.getDirection())
                            .speed(baseWindComp.getSpeed())
                            .verticalSpeed(baseWindComp.getVerticalSpeed() + verticalSpeed)
                            .build();
                });
    }
}
