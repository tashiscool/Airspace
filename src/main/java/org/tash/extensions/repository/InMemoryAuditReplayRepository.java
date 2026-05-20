package org.tash.extensions.repository;

import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.engine.OperationalDecisionAuditEnvelope;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class InMemoryAuditReplayRepository implements AuditReplayRepository {
    private final Map<String, String> json = new LinkedHashMap<>();

    @Override
    public synchronized String saveAuditEnvelope(OperationalDecisionAuditEnvelope envelope) {
        return save("audit", envelope);
    }

    @Override
    public synchronized String saveReplayBundle(OperationalDecisionReplayBundle bundle) {
        return save("replay", bundle);
    }

    @Override
    public synchronized Optional<String> findJsonById(String id) {
        return Optional.ofNullable(json.get(id));
    }

    @Override
    public synchronized List<String> listIds() {
        return new ArrayList<>(json.keySet());
    }

    private String save(String type, Object value) {
        String serialized = CanonicalJson.write(value);
        String id = type + ":" + CanonicalJson.sha256String(serialized);
        json.put(id, serialized);
        return id;
    }
}
