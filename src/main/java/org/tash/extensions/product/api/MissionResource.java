package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.QueryParam;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.ReservationWorkflowResult;

import java.util.List;

@Path("/api/missions")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class MissionResource {
    @Inject
    AirspaceProductService productService;

    @GET
    public List<ProductDtos.MissionSummary> list() {
        return productService.missions();
    }

    @POST
    public ProductDtos.MissionSummary create(ProductDtos.MissionRequest request) {
        return productService.createMission(request);
    }

    @GET
    @Path("/{id}")
    public ProductDtos.MissionDetail detail(@PathParam("id") String id) {
        return productService.mission(id);
    }

    @POST
    @Path("/{id}/lock")
    public ProductDtos.MissionSummary lock(@PathParam("id") String id, ProductDtos.MissionRequest request) {
        return productService.lockMission(id, request == null ? null : request.getActor());
    }

    @POST
    @Path("/{id}/unlock")
    public ProductDtos.MissionSummary unlock(@PathParam("id") String id, ProductDtos.MissionRequest request) {
        return productService.unlockMission(id, request == null ? null : request.getActor());
    }

    @POST
    @Path("/{id}/reservations")
    public ReservationWorkflowResult createReservation(@PathParam("id") String id, ProductDtos.ReservationRequest request) {
        return productService.createReservation(id, request);
    }

    @GET
    @Path("/{id}/weather-verdict")
    public ProductDtos.MissionWeatherVerdictSummary weatherVerdict(@PathParam("id") String id) {
        return productService.missionWeatherVerdict(id);
    }

    @GET
    @Path("/{id}/weather-changes")
    public List<ProductDtos.WeatherSourceSummary> weatherChanges(@PathParam("id") String id,
                                                                 @QueryParam("since") String since,
                                                                 @QueryParam("limit") Integer limit) {
        return productService.weatherChanges(id, since, limit);
    }

    @GET
    @Path("/{id}/route-impact")
    public ProductDtos.RouteImpactSummary routeImpact(@PathParam("id") String id,
                                                      @QueryParam("reservationId") String reservationId) {
        return productService.routeImpact(id, reservationId);
    }

    @POST
    @Path("/{id}/pireps/relevant")
    public ProductDtos.PirepRelevanceResult relevantPireps(@PathParam("id") String id,
                                                           ProductDtos.PirepRelevanceRequest request) {
        return productService.relevantPireps(id, request);
    }

    @POST
    @Path("/{id}/coordinate-weather")
    public ProductDtos.CoordinationDraftSummary coordinateWeather(@PathParam("id") String id,
                                                                  ProductDtos.CoordinationDraftRequest request) {
        return productService.coordinationDraft(id, request);
    }

    @GET
    @Path("/{id}/pilot-brief")
    public ProductDtos.PilotBriefSummary pilotBrief(@PathParam("id") String id,
                                                    @QueryParam("since") String since) {
        return productService.pilotBrief(id, since);
    }
}
