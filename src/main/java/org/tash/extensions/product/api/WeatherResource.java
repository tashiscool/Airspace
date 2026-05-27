package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.QueryParam;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.visualization.AirspaceFeatureCollection;

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

    @GET
    @Path("/live/status")
    public ProductDtos.WeatherLiveStatusSummary liveStatus() {
        return productService.liveWeatherStatus();
    }

    @POST
    @Path("/live/poll")
    public ProductDtos.WeatherLivePollSummary pollLive(ProductDtos.WeatherLivePollRequest request) {
        return productService.pollLiveWeather(request);
    }

    @GET
    @Path("/patterns")
    public List<ProductDtos.WeatherPatternSummary> patterns() {
        return productService.weatherPatterns();
    }

    @GET
    @Path("/patterns/features")
    public AirspaceFeatureCollection patternFeatures() {
        return productService.weatherPatternFeatures();
    }

    @GET
    @Path("/events")
    public List<ProductDtos.WeatherEventSummary> events() {
        return productService.weatherEvents();
    }

    @POST
    @Path("/route-sample")
    public List<ProductDtos.RouteWeatherPatternIntersectionSummary> routeSample(ProductDtos.WeatherPatternRouteSampleRequest request) {
        return productService.routeSample(request);
    }
}
