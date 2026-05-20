package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class LegacyZipComparison {
    private final LegacyZipArtifactExtractor extractor = new LegacyZipArtifactExtractor();

    public Result compare(Path older, Path newer) throws IOException {
        Map<String, byte[]> olderEntries = extractor.entries(older);
        Map<String, byte[]> newerEntries = extractor.entries(newer);
        Set<String> allNames = new LinkedHashSet<>();
        allNames.addAll(olderEntries.keySet());
        allNames.addAll(newerEntries.keySet());

        List<String> added = new ArrayList<>();
        List<String> removed = new ArrayList<>();
        List<String> changed = new ArrayList<>();
        List<String> unchanged = new ArrayList<>();
        for (String name : allNames) {
            byte[] oldBytes = olderEntries.get(name);
            byte[] newBytes = newerEntries.get(name);
            if (oldBytes == null) {
                added.add(name);
            } else if (newBytes == null) {
                removed.add(name);
            } else if (sameBytes(oldBytes, newBytes)) {
                unchanged.add(name);
            } else {
                changed.add(name);
            }
        }
        return Result.builder()
                .added(added)
                .removed(removed)
                .changed(changed)
                .unchanged(unchanged)
                .build();
    }

    private boolean sameBytes(byte[] left, byte[] right) {
        if (left.length != right.length) {
            return false;
        }
        for (int i = 0; i < left.length; i++) {
            if (left[i] != right[i]) {
                return false;
            }
        }
        return true;
    }

    @Data
    @Builder
    public static class Result {
        private List<String> added;
        private List<String> removed;
        private List<String> changed;
        private List<String> unchanged;
    }
}
