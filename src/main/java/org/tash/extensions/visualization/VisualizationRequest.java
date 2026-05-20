package org.tash.extensions.visualization;

import lombok.Data;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.weather.decision.RouteWeatherAdvisory;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.List;

@Data
public class VisualizationRequest {
    private List<AirspaceReservation> reservations = new ArrayList<>();
    private List<ReservationConflict> conflicts = new ArrayList<>();
    private List<NotamAirspaceRestriction> notamRestrictions = new ArrayList<>();
    private List<WeatherProduct> weatherProducts = new ArrayList<>();
    private List<RouteWeatherAdvisory> weatherAdvisories = new ArrayList<>();
}
