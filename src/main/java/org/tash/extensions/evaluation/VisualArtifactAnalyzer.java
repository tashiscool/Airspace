package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

public class VisualArtifactAnalyzer {
    public Analysis analyze(Path path) throws IOException {
        BufferedImage image = ImageIO.read(path.toFile());
        if (image == null) {
            throw new IOException("Unsupported visual artifact: " + path);
        }
        byte[] hash = sha256(path);
        return Analysis.builder()
                .path(path)
                .width(image.getWidth())
                .height(image.getHeight())
                .sha256(toHex(hash))
                .referenceOnly(true)
                .build();
    }

    public boolean sameImage(Path first, Path second) throws IOException {
        return analyze(first).getSha256().equals(analyze(second).getSha256());
    }

    private byte[] sha256(Path path) throws IOException {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            try (InputStream input = Files.newInputStream(path)) {
                byte[] buffer = new byte[8192];
                int read;
                while ((read = input.read(buffer)) >= 0) {
                    digest.update(buffer, 0, read);
                }
            }
            return digest.digest();
        } catch (NoSuchAlgorithmException ex) {
            throw new IllegalStateException("SHA-256 is unavailable", ex);
        }
    }

    private String toHex(byte[] bytes) {
        StringBuilder builder = new StringBuilder(bytes.length * 2);
        for (byte value : bytes) {
            builder.append(String.format("%02x", value & 0xff));
        }
        return builder.toString();
    }

    @Data
    @Builder
    public static class Analysis {
        private Path path;
        private int width;
        private int height;
        private String sha256;
        private boolean referenceOnly;
    }
}
