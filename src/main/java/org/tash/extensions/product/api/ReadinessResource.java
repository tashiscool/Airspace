package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.AirspaceReadinessService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.List;

@Path("/api")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class ReadinessResource {
    @Inject
    AirspaceReadinessService readinessService;

    @GET
    @Path("/gaps")
    public List<ProductDtos.AirspaceGapStatus> gaps() {
        return readinessService.gaps();
    }

    @GET
    @Path("/gaps/release-gates")
    public List<ProductDtos.ReleaseGateSummary> releaseGates() {
        return readinessService.releaseGates();
    }

    @GET
    @Path("/providers/status")
    public List<ProductDtos.ProviderHealthSummary> providerStatus() {
        return readinessService.providersStatus();
    }

    @POST
    @Path("/providers/weather/poll")
    public ProductDtos.ProviderHealthSummary pollProviderWeather(ProductDtos.WeatherLivePollRequest request) {
        return readinessService.pollProviderWeather(request);
    }

    @POST
    @Path("/calibration/run")
    public ProductDtos.CalibrationRunSummary runCalibration(ProductDtos.CalibrationRunRequest request) {
        return readinessService.runCalibration(request);
    }

    @GET
    @Path("/calibration/reports")
    public List<ProductDtos.CalibrationRunSummary> calibrationReports() {
        return readinessService.calibrationReports();
    }

    @GET
    @Path("/safety/dossier")
    public ProductDtos.SafetyCaseDossierSummary safetyDossier() {
        return readinessService.safetyDossier();
    }

    @GET
    @Path("/collaboration/common-operating-picture")
    public ProductDtos.CommonOperatingPictureSummary commonOperatingPicture() {
        return readinessService.commonOperatingPicture();
    }

    @GET
    @Path("/collaboration/participants")
    public List<ProductDtos.CollaborativeParticipantSummary> collaborativeParticipants() {
        return readinessService.collaborativeParticipants();
    }

    @GET
    @Path("/collaboration/proposals")
    public List<ProductDtos.CollaborativeProposalSummary> collaborativeProposals() {
        return readinessService.collaborativeProposals();
    }

    @POST
    @Path("/collaboration/proposals")
    public ProductDtos.CollaborativeProposalSummary createCollaborativeProposal(ProductDtos.CollaborativeProposalRequest request) {
        return readinessService.createCollaborativeProposal(request);
    }

    @POST
    @Path("/collaboration/proposals/{proposalId}/comment")
    public ProductDtos.CollaborativeProposalSummary commentOnCollaborativeProposal(@PathParam("proposalId") String proposalId,
                                                                                   ProductDtos.CollaborativeProposalActionRequest request) {
        return readinessService.commentOnCollaborativeProposal(proposalId, request);
    }

    @POST
    @Path("/collaboration/proposals/{proposalId}/accept")
    public ProductDtos.CollaborativeProposalSummary acceptCollaborativeProposal(@PathParam("proposalId") String proposalId,
                                                                                ProductDtos.CollaborativeProposalActionRequest request) {
        return readinessService.acceptCollaborativeProposal(proposalId, request);
    }

    @POST
    @Path("/collaboration/proposals/{proposalId}/reject")
    public ProductDtos.CollaborativeProposalSummary rejectCollaborativeProposal(@PathParam("proposalId") String proposalId,
                                                                                ProductDtos.CollaborativeProposalActionRequest request) {
        return readinessService.rejectCollaborativeProposal(proposalId, request);
    }

    @POST
    @Path("/collaboration/proposals/{proposalId}/approve")
    public ProductDtos.CollaborativeProposalSummary approveCollaborativeProposal(@PathParam("proposalId") String proposalId,
                                                                                 ProductDtos.CollaborativeProposalActionRequest request) {
        return readinessService.approveCollaborativeProposal(proposalId, request);
    }

    @POST
    @Path("/collaboration/proposals/{proposalId}/deliver")
    public ProductDtos.CollaborativeProposalSummary deliverCollaborativeProposal(@PathParam("proposalId") String proposalId,
                                                                                 ProductDtos.CollaborativeProposalActionRequest request) {
        return readinessService.deliverCollaborativeProposal(proposalId, request);
    }

    @POST
    @Path("/coordination/{draftId}/approve")
    public ProductDtos.CoordinationDeliveryStatusSummary approveCoordination(@PathParam("draftId") String draftId,
                                                                             ProductDtos.CoordinationDeliveryRequest request) {
        return readinessService.approveCoordination(draftId, request);
    }

    @POST
    @Path("/coordination/{draftId}/mark-delivered")
    public ProductDtos.CoordinationDeliveryStatusSummary markCoordinationDelivered(@PathParam("draftId") String draftId,
                                                                                   ProductDtos.CoordinationDeliveryRequest request) {
        return readinessService.markCoordinationDelivered(draftId, request);
    }
}
