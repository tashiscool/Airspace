package org.tash.extensions.http;

import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;

@Path("/api/carf")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class CarfAnalysisResource {
    private final CarfAnalysisService service = new CarfAnalysisService();

    @POST
    @Path("/analyze")
    public CarfAnalysisResult analyze(RawTextRequest request) {
        return service.parseValidateMap(request == null ? null : request.getRawText());
    }
}
