package org.tash.extensions.evaluation;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

public class ArtifactEvaluator {
    public boolean sameBytes(Path first, Path second) throws IOException {
        return Files.size(first) == Files.size(second) && sha256(first).equals(sha256(second));
    }

    public ArtifactEvaluationEntry duplicate(Path first, Path second, String category) throws IOException {
        return ArtifactEvaluationEntry.builder()
                .path(first + " == " + second)
                .status(sameBytes(first, second)
                        ? ArtifactEvaluationStatus.REFERENCE_ONLY
                        : ArtifactEvaluationStatus.UNREVIEWED)
                .category(category)
                .finding(sameBytes(first, second) ? "Duplicate artifact" : "Files differ")
                .testReference("ArtifactEvaluatorTest")
                .build();
    }

    private String sha256(Path path) throws IOException {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] buffer = new byte[8192];
            try (InputStream input = Files.newInputStream(path)) {
                int read;
                while ((read = input.read(buffer)) >= 0) {
                    digest.update(buffer, 0, read);
                }
            }
            StringBuilder builder = new StringBuilder();
            for (byte b : digest.digest()) {
                builder.append(String.format("%02x", b));
            }
            return builder.toString();
        } catch (NoSuchAlgorithmException ex) {
            throw new IllegalStateException("SHA-256 not available", ex);
        }
    }
}
