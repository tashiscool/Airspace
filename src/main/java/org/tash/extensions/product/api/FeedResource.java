package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.List;

@Path("/api/feed")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class FeedResource {
    @Inject
    AirspaceProductService productService;

    @POST
    @Path("/ingest")
    public OperationalFeedBatchResult ingest(ProductDtos.FeedIngestRequest request) {
        return productService.ingestFeed(request);
    }

    @GET
    @Path("/artifacts")
    public List<ProductDtos.FeedArtifactSummary> artifacts() {
        return productService.feedArtifacts();
    }

    @GET
    @Path("/artifacts/{id}")
    public ProductDtos.FeedArtifactSummary artifact(@PathParam("id") String id) {
        return productService.feedArtifact(id);
    }

    @GET
    @Path("/artifacts/{id}/transactions")
    public List<ProductDtos.FeedTransactionSummary> transactions(@PathParam("id") String id) {
        return productService.feedTransactions(id);
    }
}
