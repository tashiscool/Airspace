package org.tash.extensions.product.dto;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.List;

public final class AuthDtos {
    private AuthDtos() {
    }

    @Data
    public static class LoginRequest {
        private String username;
        private String password;
    }

    @Data
    @Builder
    public static class SessionResponse {
        private boolean accepted;
        private String token;
        private UserSummary user;
        @Builder.Default
        private List<String> diagnostics = new ArrayList<>();
    }

    @Data
    @Builder
    public static class UserSummary {
        private String id;
        private String username;
        private String displayName;
        @Builder.Default
        private List<String> roles = new ArrayList<>();
    }
}
