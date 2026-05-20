package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.List;

@Path("/api/messages")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class MessageResource {
    @Inject
    AirspaceProductService productService;

    @GET
    public List<ProductDtos.MessageSummary> list() {
        return productService.messages();
    }

    @GET
    @Path("/{id}")
    public ProductDtos.MessageSummary get(@PathParam("id") String id) {
        return productService.message(id);
    }

    @POST
    @Path("/send")
    public ProductDtos.MessageSummary send(ProductDtos.MessageRequest request) {
        return productService.sendMessage(request);
    }

    @POST
    @Path("/{id}/reply")
    public ProductDtos.MessageSummary reply(@PathParam("id") String id, ProductDtos.MessageRequest request) {
        return productService.replyMessage(id, request);
    }

    @POST
    @Path("/{id}/forward")
    public ProductDtos.MessageSummary forward(@PathParam("id") String id, ProductDtos.MessageRequest request) {
        return productService.forwardMessage(id, request);
    }
}
