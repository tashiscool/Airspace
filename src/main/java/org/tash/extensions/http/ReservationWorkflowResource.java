package org.tash.extensions.http;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.PUT;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceVisualizationService;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.ReservationWorkflowRecord;
import org.tash.extensions.workflow.ReservationWorkflowResult;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.util.Collections;

@Path("/api/reservations")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class ReservationWorkflowResource {
    @Inject
    ReservationWorkflowService workflowService;
    @Inject
    AirspaceProductService productService;
    @Inject
    AirspaceVisualizationService visualizationService;

    @POST
    @Path("/drafts")
    public ReservationWorkflowResult createDraft(RawTextRequest request) {
        return workflowService.createDraft(request == null ? null : request.getRawText(),
                actor(request));
    }

    @PUT
    @Path("/{id}")
    public ReservationWorkflowResult updateDraft(@PathParam("id") String id, RawTextRequest request) {
        return productService.updateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/validate")
    public ReservationWorkflowResult validate(@PathParam("id") String id, RawTextRequest request) {
        return productService.validateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/parse")
    public ReservationWorkflowResult parse(@PathParam("id") String id, RawTextRequest request) {
        return productService.validateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/deconflict")
    public ReservationWorkflowResult deconflict(@PathParam("id") String id, RawTextRequest request) {
        return productService.validateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/force-parse")
    public ReservationWorkflowResult forceParse(@PathParam("id") String id, RawTextRequest request) {
        return productService.validateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/force-deconflict")
    public ReservationWorkflowResult forceDeconflict(@PathParam("id") String id, RawTextRequest request) {
        return productService.validateReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/submit")
    public ReservationWorkflowResult submit(@PathParam("id") String id, RawTextRequest request) {
        return productService.submitReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/approve")
    public ReservationWorkflowResult approve(@PathParam("id") String id, RawTextRequest request) {
        return productService.approveReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/reject")
    public ReservationWorkflowResult reject(@PathParam("id") String id, RawTextRequest request) {
        return productService.rejectReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/cancel")
    public ReservationWorkflowResult cancel(@PathParam("id") String id, RawTextRequest request) {
        return productService.cancelReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/complete")
    public ReservationWorkflowResult complete(@PathParam("id") String id, RawTextRequest request) {
        return productService.completeReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/lock")
    public ReservationWorkflowResult lock(@PathParam("id") String id, RawTextRequest request) {
        return productService.lockReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/unlock")
    public ReservationWorkflowResult unlock(@PathParam("id") String id, RawTextRequest request) {
        return productService.unlockReservation(id, reservationRequest(request));
    }

    @POST
    @Path("/{id}/conflicts/{conflictId}/review")
    public ReservationWorkflowResult reviewConflict(@PathParam("id") String id,
                                                    @PathParam("conflictId") String conflictId,
                                                    ConflictReviewRequest request) {
        return workflowService.reviewConflict(id, conflictId,
                request != null && request.isAccepted(),
                request == null ? null : request.getReviewer(),
                request == null ? null : request.getNote());
    }

    @GET
    @Path("/{id}/features")
    public AirspaceFeatureCollection features(@PathParam("id") String id) {
        ReservationWorkflowRecord record = workflowService.findById(id);
        if (record.getLastAnalysis() == null) {
            return AirspaceFeatureCollection.builder().features(Collections.emptyList()).build();
        }
        AirspaceFeatureCollection reservations =
                visualizationService.featuresForReservations(record.getLastAnalysis().getReservations());
        reservations.getFeatures().addAll(
                visualizationService.featuresForConflicts(record.getLastAnalysis().getConflicts()).getFeatures());
        return reservations;
    }

    private String actor(RawTextRequest request) {
        return request == null || request.getActor() == null ? "system" : request.getActor();
    }

    private ProductDtos.ReservationRequest reservationRequest(RawTextRequest request) {
        ProductDtos.ReservationRequest out = new ProductDtos.ReservationRequest();
        if (request != null) {
            out.setRawText(request.getRawText());
            out.setActor(request.getActor());
            out.setNote(request.getNote());
        }
        return out;
    }
}
