package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.weather.pirep.PirepReport;
import org.tash.extensions.weather.decision.RouteImpactCalibrationModel;
import org.tash.extensions.weather.decision.RouteDemandSnapshot;
import org.tash.extensions.weather.decision.SectorDemandSnapshot;
import org.tash.extensions.weather.product.EnsembleWeatherProduct;
import org.tash.extensions.weather.product.WeatherProduct;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class OperationalDecisionRequest {
    @Builder.Default private final List<String> rawUsnsMessages = new ArrayList<>();
    @Builder.Default private final List<String> rawCarfMessages = new ArrayList<>();
    @Builder.Default private final List<WeatherProduct> weatherProducts = new ArrayList<>();
    @Builder.Default private final List<PirepReport> pireps = new ArrayList<>();
    @Builder.Default private final List<NotamAirspaceRestriction> notamRestrictions = new ArrayList<>();
    @Builder.Default private final List<AirspaceReservation> reservations = new ArrayList<>();
    @Builder.Default private final List<GeoCoordinate> route = new ArrayList<>();
    @Builder.Default private final List<EnsembleWeatherProduct> ensembleProducts = new ArrayList<>();
    @Builder.Default private final List<SectorDemandSnapshot> sectorDemand = new ArrayList<>();
    @Builder.Default private final List<RouteDemandSnapshot> routeDemand = new ArrayList<>();
    private final EngineConfig engineConfig;
    private final RouteImpactCalibrationModel calibrationModel;
    private final String auditSigningKeyId;
    private final CarfReferenceDataProvider referenceDataProvider;
    private final ZonedDateTime decisionTime;
}
