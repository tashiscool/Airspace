package org.tash.extensions.product;

import org.junit.jupiter.api.Test;
import org.tash.extensions.product.auth.AuthService;
import org.tash.extensions.product.dto.AuthDtos;

import static org.junit.jupiter.api.Assertions.*;

class AuthServiceTest {
    @Test
    void localSessionsAuthenticateAndLogoutDeterministically() {
        AuthService service = new AuthService();
        AuthDtos.LoginRequest request = new AuthDtos.LoginRequest();
        request.setUsername("planner");
        request.setPassword("planner");

        AuthDtos.SessionResponse accepted = service.login(request);

        assertTrue(accepted.isAccepted());
        assertNotNull(accepted.getToken());
        assertTrue(service.isAuthenticated("Bearer " + accepted.getToken()));
        assertEquals("planner", service.me("Bearer " + accepted.getToken()).getUsername());

        service.logout("Bearer " + accepted.getToken());
        assertFalse(service.isAuthenticated("Bearer " + accepted.getToken()));
        assertNull(service.me("Bearer " + accepted.getToken()));
    }

    @Test
    void badCredentialsReturnDiagnosticsWithoutSession() {
        AuthService service = new AuthService();
        AuthDtos.LoginRequest request = new AuthDtos.LoginRequest();
        request.setUsername("planner");
        request.setPassword("wrong");

        AuthDtos.SessionResponse rejected = service.login(request);

        assertFalse(rejected.isAccepted());
        assertNull(rejected.getToken());
        assertEquals("Invalid username or password", rejected.getDiagnostics().get(0));
        assertFalse(service.isAuthenticated(null));
    }
}
