package org.tash.extensions.weather.coordination;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class WeatherCoordinationResult {
    private final WeatherCoordinationAdvisory advisory;
    @Builder.Default
    private final List<WeatherDeskReviewItem> reviewItems = new ArrayList<>();
    @Builder.Default
    private final List<OperationalWeatherConstraint> constraints = new ArrayList<>();
    @Builder.Default
    private final List<ControllerHandoffNote> handoffNotes = new ArrayList<>();

    public List<WeatherDeskReviewItem> getReviewItems() {
        return Collections.unmodifiableList(reviewItems);
    }

    public List<OperationalWeatherConstraint> getConstraints() {
        return Collections.unmodifiableList(constraints);
    }

    public List<ControllerHandoffNote> getHandoffNotes() {
        return Collections.unmodifiableList(handoffNotes);
    }
}
