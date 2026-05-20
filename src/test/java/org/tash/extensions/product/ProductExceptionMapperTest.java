package org.tash.extensions.product;

import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.Test;
import org.tash.extensions.product.api.ProductExceptionMapper;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class ProductExceptionMapperTest {
    @Test
    void illegalArgumentBecomesOperatorFriendlyNotFoundResponse() {
        Response response = new ProductExceptionMapper().toResponse(new IllegalArgumentException("Unknown mission: MISSING"));

        assertEquals(404, response.getStatus());
        assertEquals("application/json", response.getMediaType().toString());
        assertInstanceOf(Map.class, response.getEntity());
        Map<?, ?> body = (Map<?, ?>) response.getEntity();
        assertEquals(false, body.get("accepted"));
        assertEquals(404, body.get("status"));
        assertTrue(String.valueOf(body.get("diagnostics")).contains("Unknown mission: MISSING"));
    }
}
