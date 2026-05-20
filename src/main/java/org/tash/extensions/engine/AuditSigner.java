package org.tash.extensions.engine;

public interface AuditSigner {
    String keyId();

    String sign(String payload);

    boolean verify(String payload, String signature);
}
