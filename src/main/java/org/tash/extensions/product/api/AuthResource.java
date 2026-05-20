package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.HeaderParam;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.product.auth.AuthService;
import org.tash.extensions.product.dto.AuthDtos;

@Path("/api/auth")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class AuthResource {
    @Inject
    AuthService authService;

    @POST
    @Path("/login")
    public AuthDtos.SessionResponse login(AuthDtos.LoginRequest request) {
        return authService.login(request);
    }

    @POST
    @Path("/logout")
    public void logout(@HeaderParam("Authorization") String authorization) {
        authService.logout(authorization);
    }

    @GET
    @Path("/me")
    public AuthDtos.UserSummary me(@HeaderParam("Authorization") String authorization) {
        return authService.me(authorization);
    }
}
