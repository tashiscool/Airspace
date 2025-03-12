package org.tash.extensions.weather.wind;

import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.Vector3;
import org.tash.extensions.weather.WindComponent;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * A wind field that interpolates between multiple wind field models based on time
 */
@Data
public class InterpolatedWindField extends AbstractWindField {
    /**
     * List of wind field models with their validity times
     */
    private final List<WindFieldModel> windFields;

    /**
     * Create a new interpolated wind field
     *
     * @param id           Wind field identifier
     * @param validityTime Validity time
     * @param windFields   Wind fields to interpolate between
     */
    public InterpolatedWindField(
            String id,
            ZonedDateTime validityTime,
            List<WindFieldModel> windFields) {
        super(id, validityTime);
        this.windFields = new ArrayList<>(windFields);
    }

    @Override
    public WindComponent getWindAt(GeoCoordinate location, ZonedDateTime time) {
        // Find the two wind fields that bracket the requested time
        WindFieldModel before = null;
        WindFieldModel after = null;

        for (WindFieldModel field : windFields) {
            if (field.getValidityTime().isBefore(time) || field.getValidityTime().equals(time)) {
                if (before == null || field.getValidityTime().isAfter(before.getValidityTime())) {
                    before = field;
                }
            }

            if (field.getValidityTime().isAfter(time) || field.getValidityTime().equals(time)) {
                if (after == null || field.getValidityTime().isBefore(after.getValidityTime())) {
                    after = field;
                }
            }
        }

        // If time is exactly at one of the wind fields, use that
        if (before != null && before.getValidityTime().equals(time)) {
            return before.getWindAt(location, time);
        }

        if (after != null && after.getValidityTime().equals(time)) {
            return after.getWindAt(location, time);
        }

        // If we don't have bracketing wind fields, use whatever we have
        if (before == null) {
            return after != null ? after.getWindAt(location, time) :
                    WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build();
        }

        if (after == null) {
            return before.getWindAt(location, time);
        }

        // Interpolate between the two wind fields
        WindComponent windBefore = before.getWindAt(location, time);
        WindComponent windAfter = after.getWindAt(location, time);

        // Calculate interpolation weight
        double totalMillis = java.time.Duration.between(before.getValidityTime(), after.getValidityTime()).toMillis();
        double elapsedMillis = java.time.Duration.between(before.getValidityTime(), time).toMillis();
        double weight = totalMillis > 0 ? elapsedMillis / totalMillis : 0;

        // Convert to vectors for interpolation
        Vector3 vBefore = windBefore.toVector();
        Vector3 vAfter = windAfter.toVector();

        // Linear interpolation
        Vector3 v = vBefore.multiply(1 - weight).add(vAfter.multiply(weight));

        return v.toWindComponent();
    }

    @Override
    public boolean isValidAt(ZonedDateTime time) {
        // Check if time is within the range of wind fields
        if (windFields.isEmpty()) {
            return false;
        }

        ZonedDateTime earliest = windFields.stream()
                .map(WindFieldModel::getValidityTime)
                .min(ZonedDateTime::compareTo)
                .orElse(validityTime);

        ZonedDateTime latest = windFields.stream()
                .map(WindFieldModel::getValidityTime)
                .max(ZonedDateTime::compareTo)
                .orElse(validityTime);

        return !time.isBefore(earliest) && !time.isAfter(latest);
    }
}
