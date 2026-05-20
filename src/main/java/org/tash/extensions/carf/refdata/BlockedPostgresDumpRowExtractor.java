package org.tash.extensions.carf.refdata;

import java.nio.file.Path;
import java.util.Collections;

public class BlockedPostgresDumpRowExtractor implements PostgresDumpRowExtractor {
    @Override
    public Extraction extract(Path dumpPath) {
        return new Extraction(false,
                "PostgreSQL custom dump row extraction requires pg_restore-compatible tooling",
                Collections.emptyMap());
    }
}
