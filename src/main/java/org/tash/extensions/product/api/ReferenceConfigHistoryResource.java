package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.QueryParam;
import jakarta.ws.rs.core.MediaType;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Path("/api")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class ReferenceConfigHistoryResource {
    @Inject
    AirspaceProductService productService;

    @GET
    @Path("/reference/navaids")
    public Map<String, GeoCoordinate> navaids() {
        return productService.referenceMap("NAVAID");
    }

    @GET
    @Path("/reference/fixes")
    public Map<String, GeoCoordinate> fixes() {
        return productService.referenceMap("FIX");
    }

    @GET
    @Path("/reference/points")
    public List<ProductDtos.ReferencePointSummary> referencePoints(@QueryParam("type") String type) {
        return productService.referencePoints(type);
    }

    @POST
    @Path("/reference/points")
    public ProductDtos.ReferencePointSummary createReferencePoint(ProductDtos.ReferencePointRequest request) {
        return productService.createReferencePoint(request);
    }

    @POST
    @Path("/reference/import/preview")
    public ProductDtos.ReferenceDataImportResult previewReferenceData(ProductDtos.ReferenceDataImportRequest request) {
        return productService.previewReferenceData(request);
    }

    @POST
    @Path("/reference/import/apply")
    public ProductDtos.ReferenceDataImportResult applyReferenceData(ProductDtos.ReferenceDataImportRequest request) {
        return productService.applyReferenceData(request);
    }

    @GET
    @Path("/config")
    public Map<String, Object> config() {
        Map<String, Object> values = new LinkedHashMap<>();
        values.put("mapStack", "OpenLayers");
        values.put("backendRuntime", "Quarkus");
        values.put("authModel", "local-rbac");
        values.put("enabledPillars", Arrays.asList("operations", "decision-engine", "weather-safety"));
        return values;
    }

    @GET
    @Path("/history")
    public List<ProductDtos.HistoryEventSummary> history() {
        return productService.history();
    }

    @GET
    @Path("/search")
    public List<ProductDtos.SearchResultSummary> search(@QueryParam("q") String query) {
        return productService.search(query);
    }

    @GET
    @Path("/metrics")
    public Map<String, Double> metrics() {
        return productService.metrics();
    }
}
