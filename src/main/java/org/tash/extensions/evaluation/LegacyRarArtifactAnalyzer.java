package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import java.util.Arrays;

public class LegacyRarArtifactAnalyzer {
    private static final byte[] RAR4_SIGNATURE = new byte[]{0x52, 0x61, 0x72, 0x21, 0x1a, 0x07, 0x00};
    private static final int MAIN_HEADER_FLAGS_OFFSET = 10;
    private static final int MHD_PASSWORD = 0x0080;

    public Analysis analyze(byte[] bytes) {
        boolean rar = bytes != null && bytes.length >= 13
                && Arrays.equals(RAR4_SIGNATURE, Arrays.copyOf(bytes, RAR4_SIGNATURE.length));
        int flags = 0;
        if (rar) {
            flags = (bytes[MAIN_HEADER_FLAGS_OFFSET] & 0xff)
                    | ((bytes[MAIN_HEADER_FLAGS_OFFSET + 1] & 0xff) << 8);
        }
        return Analysis.builder()
                .rar(rar)
                .mainHeaderFlags(flags)
                .encryptedHeaders(rar && (flags & MHD_PASSWORD) != 0)
                .build();
    }

    @Data
    @Builder
    public static class Analysis {
        private boolean rar;
        private int mainHeaderFlags;
        private boolean encryptedHeaders;
    }
}
