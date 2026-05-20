package org.tash.extensions.http;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceVisualizationService;
import org.tash.extensions.visualization.VisualizationRequest;

@Path("/api")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class VisualizationResource {
    @Inject
    AirspaceVisualizationService service;

    @POST
    @Path("/visualize")
    public AirspaceFeatureCollection visualize(VisualizationRequest request) {
        return service.combined(request);
    }
}
