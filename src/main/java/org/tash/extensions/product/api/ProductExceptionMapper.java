package org.tash.extensions.product.api;

import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import jakarta.ws.rs.ext.ExceptionMapper;
import jakarta.ws.rs.ext.Provider;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

@Provider
public class ProductExceptionMapper implements ExceptionMapper<IllegalArgumentException> {
    @Override
    public Response toResponse(IllegalArgumentException exception) {
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("accepted", false);
        body.put("status", 404);
        body.put("diagnostics", Collections.singletonList(exception.getMessage()));
        return Response.status(Response.Status.NOT_FOUND)
                .type(MediaType.APPLICATION_JSON_TYPE)
                .entity(body)
                .build();
    }
}
