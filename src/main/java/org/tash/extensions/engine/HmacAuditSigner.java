package org.tash.extensions.engine;

import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;
import java.nio.charset.StandardCharsets;
import java.util.Base64;

public class HmacAuditSigner implements AuditSigner {
    private final String keyId;
    private final byte[] secret;

    public HmacAuditSigner(String keyId, String secret) {
        this.keyId = keyId == null ? "local-test-key" : keyId;
        this.secret = (secret == null ? "airspace-local-audit-secret" : secret).getBytes(StandardCharsets.UTF_8);
    }

    @Override
    public String keyId() {
        return keyId;
    }

    @Override
    public String sign(String payload) {
        try {
            Mac mac = Mac.getInstance("HmacSHA256");
            mac.init(new SecretKeySpec(secret, "HmacSHA256"));
            return Base64.getEncoder().encodeToString(mac.doFinal((payload == null ? "" : payload).getBytes(StandardCharsets.UTF_8)));
        } catch (Exception e) {
            throw new IllegalStateException("Unable to sign audit payload", e);
        }
    }

    @Override
    public boolean verify(String payload, String signature) {
        return sign(payload).equals(signature);
    }
}
