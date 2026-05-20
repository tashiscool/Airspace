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

@Path("/api/reservations/{reservationId}/supplements")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class ReservationSupplementResource {
    @Inject
    AirspaceProductService productService;

    @GET
    public List<ProductDtos.ReservationSupplementSummary> list(@PathParam("reservationId") String reservationId) {
        return productService.supplements(reservationId);
    }

    @POST
    public ProductDtos.ReservationSupplementSummary create(@PathParam("reservationId") String reservationId,
                                                           ProductDtos.ReservationSupplementRequest request) {
        return productService.createSupplement(reservationId, request);
    }

    @POST
    @Path("/{supplementId}/transition")
    public ProductDtos.ReservationSupplementSummary transition(@PathParam("supplementId") String supplementId,
                                                               ProductDtos.SupplementTransitionRequest request) {
        return productService.transitionSupplement(supplementId, request);
    }
}
