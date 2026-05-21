package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.QueryParam;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.List;

@Path("/api/weather")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class WeatherResource {
    @Inject
    AirspaceProductService productService;

    @GET
    @Path("/affected-missions")
    public List<ProductDtos.AffectedMissionSummary> affectedMissions(@QueryParam("sourceId") String sourceId,
                                                                     @QueryParam("limit") Integer limit) {
        return productService.affectedMissions(sourceId, limit);
    }
}
