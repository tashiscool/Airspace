package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.messaging.transaction.FdcAcknowledgementCommand;
import org.tash.extensions.messaging.transaction.ServiceRequestCommand;
import org.tash.extensions.messaging.transaction.ServiceTableCommand;
import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.NotamFieldParseResult;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.product.WeatherProductParseResult;

import java.util.List;

@Data
@Builder
public class UsnsTransactionIngestResult {
    private UsnsTransaction transaction;
    private boolean supported;
    private DomesticNotamParseResult domesticResult;
    private CarfAnalysisResult carfAnalysisResult;
    private CarfMessageFamilyParseResult familyParseResult;
    private UsnsRoutingOutcome routingOutcome;
    private CarfRouteMessage carfMessage;
    private NotamAirspaceRestriction fdcLaserRestriction;
    private NotamFieldParseResult notamFieldResult;
    private FdcAcknowledgementCommand fdcAcknowledgement;
    private ServiceRequestCommand serviceRequest;
    private ServiceTableCommand serviceTable;
    private WeatherProductParseResult weatherProductResult;
    private PirepIngestResult pirepIngestResult;
    private List<String> warnings;
    private List<String> errors;
}
