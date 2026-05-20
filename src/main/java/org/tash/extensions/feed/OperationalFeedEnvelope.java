package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.time.ZonedDateTime;
import java.util.LinkedHashMap;
import java.util.Map;

@Data
@Builder(toBuilder = true)
public class OperationalFeedEnvelope {
    private final String id;
    private final String sourceId;
    private final OperationalFeedType type;
    private final ZonedDateTime receivedAt;
    private final String rawPayload;
    @Builder.Default
    private final Map<String, String> metadata = new LinkedHashMap<>();

    public String payloadHash() {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] bytes = digest.digest((rawPayload == null ? "" : rawPayload).getBytes(StandardCharsets.UTF_8));
            StringBuilder out = new StringBuilder();
            for (byte b : bytes) {
                out.append(String.format("%02x", b));
            }
            return out.toString();
        } catch (Exception ex) {
            throw new IllegalStateException("Unable to hash feed payload", ex);
        }
    }
}
