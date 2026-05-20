package org.tash.extensions.product.auth;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.extensions.product.dto.AuthDtos;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Base64;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.UUID;

@ApplicationScoped
public class AuthService {
    private final Map<String, AuthDtos.UserSummary> users = new LinkedHashMap<>();
    private final Map<String, String> passwordHashes = new LinkedHashMap<>();
    private final Map<String, String> sessions = new LinkedHashMap<>();

    public AuthService() {
        addUser("planner", "planner", "CARF Planner", "PLANNER", "OPERATOR");
        addUser("supervisor", "supervisor", "Operations Supervisor", "SUPERVISOR", "OPERATOR");
        addUser("admin", "admin", "Airspace Administrator", "ADMIN", "SUPERVISOR", "PLANNER", "OPERATOR");
    }

    public AuthDtos.SessionResponse login(AuthDtos.LoginRequest request) {
        if (request == null || request.getUsername() == null) {
            return rejected("Username is required");
        }
        String username = request.getUsername().trim().toLowerCase(java.util.Locale.US);
        AuthDtos.UserSummary user = users.get(username);
        if (user == null || !passwordHashes.get(username).equals(hash(request.getPassword()))) {
            return rejected("Invalid username or password");
        }
        String token = "local-" + UUID.randomUUID() + "-" + ZonedDateTime.now(ZoneOffset.UTC).toEpochSecond();
        sessions.put(token, username);
        return AuthDtos.SessionResponse.builder()
                .accepted(true)
                .token(token)
                .user(user)
                .diagnostics(Collections.emptyList())
                .build();
    }

    public void logout(String token) {
        if (token != null) {
            sessions.remove(token.replace("Bearer ", ""));
        }
    }

    public AuthDtos.UserSummary me(String token) {
        if (token == null) {
            return null;
        }
        return users.get(sessions.get(token.replace("Bearer ", "")));
    }

    public boolean isAuthenticated(String token) {
        return me(token) != null;
    }

    public boolean hasAnyRole(String token, String... roles) {
        AuthDtos.UserSummary user = me(token);
        if (user == null) {
            return false;
        }
        if (roles == null || roles.length == 0) {
            return true;
        }
        for (String role : roles) {
            if (user.getRoles().contains(role)) {
                return true;
            }
        }
        return false;
    }

    private void addUser(String username, String password, String displayName, String... roles) {
        AuthDtos.UserSummary user = AuthDtos.UserSummary.builder()
                .id(UUID.nameUUIDFromBytes(username.getBytes(StandardCharsets.UTF_8)).toString())
                .username(username)
                .displayName(displayName)
                .roles(Arrays.asList(roles))
                .build();
        users.put(username, user);
        passwordHashes.put(username, hash(password));
    }

    private AuthDtos.SessionResponse rejected(String diagnostic) {
        return AuthDtos.SessionResponse.builder()
                .accepted(false)
                .diagnostics(Collections.singletonList(diagnostic))
                .build();
    }

    private String hash(String value) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            return Base64.getEncoder().encodeToString(digest.digest((value == null ? "" : value).getBytes(StandardCharsets.UTF_8)));
        } catch (Exception ex) {
            throw new IllegalStateException("Unable to hash password", ex);
        }
    }
}
