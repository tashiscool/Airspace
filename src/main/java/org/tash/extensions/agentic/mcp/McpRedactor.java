package org.tash.extensions.agentic.mcp;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@ApplicationScoped
public class McpRedactor {
    private static final ObjectMapper MAPPER = new ObjectMapper().registerModule(new JavaTimeModule());

    public McpRedactionResult redact(Object value) {
        Object normalized;
        try {
            normalized = MAPPER.convertValue(value, Object.class);
        } catch (IllegalArgumentException ex) {
            normalized = String.valueOf(value);
        }
        RedactionState state = new RedactionState();
        Object redacted = redactValue(null, normalized, state);
        return McpRedactionResult.builder()
                .value(redacted)
                .status(state.changed ? "REDACTED" : "CLEAN")
                .build();
    }

    @SuppressWarnings("unchecked")
    private Object redactValue(String key, Object value, RedactionState state) {
        if (value == null) {
            return null;
        }
        if (sensitiveKey(key)) {
            state.changed = true;
            return "[REDACTED]";
        }
        if (value instanceof Map<?, ?>) {
            Map<?, ?> map = (Map<?, ?>) value;
            Map<String, Object> redacted = new LinkedHashMap<>();
            for (Map.Entry<?, ?> entry : map.entrySet()) {
                String childKey = String.valueOf(entry.getKey());
                redacted.put(childKey, redactValue(childKey, entry.getValue(), state));
            }
            return redacted;
        }
        if (value instanceof List<?>) {
            List<?> list = (List<?>) value;
            List<Object> redacted = new ArrayList<>();
            for (Object item : list) {
                redacted.add(redactValue(key, item, state));
            }
            return redacted;
        }
        if (value instanceof String && ((String) value).length() > 600) {
            String text = (String) value;
            state.changed = true;
            return text.substring(0, 600) + "...[TRUNCATED]";
        }
        return value;
    }

    private boolean sensitiveKey(String key) {
        if (key == null) {
            return false;
        }
        String normalized = key.replace("_", "").replace("-", "").toLowerCase(Locale.US);
        return normalized.contains("token")
                || normalized.contains("password")
                || normalized.contains("secret")
                || normalized.contains("apikey")
                || normalized.contains("authorization")
                || normalized.contains("credential")
                || normalized.contains("rawpayload")
                || normalized.contains("rawtext")
                || normalized.contains("rawbody");
    }

    private static class RedactionState {
        boolean changed;
    }
}
