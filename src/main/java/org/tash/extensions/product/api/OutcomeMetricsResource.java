package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.simulation.OperationalSimulationService;
import org.tash.extensions.simulation.OutcomeMetricsReport;
import org.tash.extensions.simulation.OutcomeMetricsRequest;

@Path("/api/outcomes")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class OutcomeMetricsResource {
    @Inject
    OperationalSimulationService simulationService;

    @GET
    @Path("/metrics")
    public OutcomeMetricsReport metrics() {
        return simulationService.outcomeMetrics(OutcomeMetricsRequest.builder()
                .includeTfmBoard(true)
                .build());
    }

    @POST
    @Path("/metrics")
    public OutcomeMetricsReport metrics(OutcomeMetricsRequest request) {
        return simulationService.outcomeMetrics(request);
    }
}
