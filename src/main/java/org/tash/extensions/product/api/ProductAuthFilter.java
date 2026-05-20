package org.tash.extensions.product.api;

import jakarta.annotation.Priority;
import jakarta.inject.Inject;
import jakarta.ws.rs.Priorities;
import jakarta.ws.rs.container.ContainerRequestContext;
import jakarta.ws.rs.container.ContainerRequestFilter;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import jakarta.ws.rs.ext.Provider;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.extensions.product.auth.AuthService;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

@Provider
@Priority(Priorities.AUTHENTICATION)
public class ProductAuthFilter implements ContainerRequestFilter {
    @Inject
    AuthService authService;
    @ConfigProperty(name = "airspace.product.auth.required", defaultValue = "false")
    boolean authRequired;

    @Override
    public void filter(ContainerRequestContext requestContext) {
        if (!authRequired || isPublicPath(requestContext.getUriInfo().getPath())) {
            return;
        }
        String authorization = requestContext.getHeaderString("Authorization");
        if (!authService.isAuthenticated(authorization)) {
            abort(requestContext, Response.Status.UNAUTHORIZED, "Authentication is required");
            return;
        }
        String[] roles = requiredRoles(requestContext.getMethod(), requestContext.getUriInfo().getPath());
        if (!authService.hasAnyRole(authorization, roles)) {
            abort(requestContext, Response.Status.FORBIDDEN, "Insufficient role for operation");
        }
    }

    private void abort(ContainerRequestContext requestContext, Response.Status status, String diagnostic) {
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("accepted", false);
        body.put("status", status.getStatusCode());
        body.put("diagnostics", Collections.singletonList(diagnostic));
        requestContext.abortWith(Response.status(status)
                .type(MediaType.APPLICATION_JSON_TYPE)
                .entity(body)
                .build());
    }

    private boolean isPublicPath(String path) {
        return "api/auth/login".equals(path);
    }

    private String[] requiredRoles(String method, String path) {
        String safePath = path == null ? "" : path;
        String safeMethod = method == null ? "GET" : method;
        if ("GET".equals(safeMethod)) {
            return new String[]{"OPERATOR", "PLANNER", "SUPERVISOR", "ADMIN"};
        }
        if (safePath.contains("/approve") || safePath.contains("/reject")) {
            return new String[]{"SUPERVISOR", "ADMIN"};
        }
        if (safePath.startsWith("api/config") || safePath.startsWith("api/reference")) {
            return new String[]{"ADMIN"};
        }
        if (safePath.startsWith("api/reservations") || safePath.startsWith("api/missions")
                || safePath.startsWith("api/messages") || safePath.startsWith("api/feed")
                || safePath.startsWith("api/decisions")) {
            return new String[]{"PLANNER", "SUPERVISOR", "ADMIN"};
        }
        return new String[]{"OPERATOR", "PLANNER", "SUPERVISOR", "ADMIN"};
    }
}
