package org.tash.extensions.evaluation;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

public interface PostgresDumpRowExtractor {
    Extraction extract(Path dumpPath) throws IOException;

    class Extraction {
        private final boolean available;
        private final String diagnostic;
        private final Map<String, Integer> rowCounts;

        public Extraction(boolean available, String diagnostic, Map<String, Integer> rowCounts) {
            this.available = available;
            this.diagnostic = diagnostic;
            this.rowCounts = rowCounts;
        }

        public boolean isAvailable() {
            return available;
        }

        public String getDiagnostic() {
            return diagnostic;
        }

        public Map<String, Integer> getRowCounts() {
            return rowCounts;
        }
    }
}
