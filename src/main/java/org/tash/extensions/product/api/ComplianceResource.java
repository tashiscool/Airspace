package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.LicenseComplianceService;
import org.tash.extensions.product.dto.ComplianceDtos;

import java.util.List;

@Path("/api/compliance")
@Produces(MediaType.APPLICATION_JSON)
@Consumes(MediaType.APPLICATION_JSON)
public class ComplianceResource {
    @Inject
    LicenseComplianceService complianceService;

    @GET
    @Path("/policy")
    public ComplianceDtos.CompliancePolicySummary policy() {
        return complianceService.policy();
    }

    @GET
    @Path("/manifest")
    public ComplianceDtos.ComplianceManifest manifest() {
        return complianceService.manifest();
    }

    @POST
    @Path("/attestations")
    public ComplianceDtos.ComplianceAttestationSummary attest(ComplianceDtos.ComplianceAttestationRequest request) {
        return complianceService.attest(request);
    }

    @GET
    @Path("/attestations")
    public List<ComplianceDtos.ComplianceAttestationSummary> attestations() {
        return complianceService.attestations();
    }
}
