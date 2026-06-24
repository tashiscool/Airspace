package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.simulation.OperationalSimulationService;
import org.tash.extensions.simulation.TfmCommandCenterRequest;
import org.tash.extensions.simulation.TfmCommandCenterSummary;

@Path("/api/tfm")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class TfmResource {
    @Inject
    OperationalSimulationService simulationService;

    @GET
    @Path("/board")
    public TfmCommandCenterSummary board() {
        return simulationService.tfmCommandCenterBoard(TfmCommandCenterRequest.builder().build());
    }

    @POST
    @Path("/board")
    public TfmCommandCenterSummary board(TfmCommandCenterRequest request) {
        return simulationService.tfmCommandCenterBoard(request);
    }
}
