package org.tash.extensions.repository;

import org.tash.extensions.engine.OperationalDecisionAuditEnvelope;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

public class JsonFileAuditReplayRepository implements AuditReplayRepository {
    private final JsonArtifactStore store;

    public JsonFileAuditReplayRepository(Path path) {
        this.store = new JsonArtifactStore(path);
    }

    @Override
    public String saveAuditEnvelope(OperationalDecisionAuditEnvelope envelope) {
        return store.save("audit", envelope);
    }

    @Override
    public String saveReplayBundle(OperationalDecisionReplayBundle bundle) {
        return store.save("replay", bundle);
    }

    @Override
    public Optional<String> findJsonById(String id) {
        return store.findJsonById(id);
    }

    @Override
    public List<String> listIds() {
        return store.listIds();
    }
}
