package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class ConstraintFusionRequest {
    @Builder.Default private final List<AirspaceReservation> reservations = new ArrayList<>();
    @Builder.Default private final List<ReservationConflict> conflicts = new ArrayList<>();
    @Builder.Default private final List<NotamAirspaceRestriction> notamRestrictions = new ArrayList<>();
    @Builder.Default private final List<WeatherProduct> weatherProducts = new ArrayList<>();
    @Builder.Default private final List<PirepIngestResult> pirepResults = new ArrayList<>();
    @Builder.Default private final List<RouteBlockagePrediction> routeBlockages = new ArrayList<>();
}
