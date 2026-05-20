package org.tash.extensions.repository;

import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.engine.OperationalDecisionResult;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class InMemoryOperationalDecisionRepository implements OperationalDecisionRepository {
    private final Map<String, OperationalDecisionResult> records = new LinkedHashMap<>();
    private final Map<String, String> json = new LinkedHashMap<>();

    @Override
    public synchronized String save(OperationalDecisionResult result) {
        String id = "decision:" + CanonicalJson.sha256(result == null ? "" : result.getDecisionSummary());
        records.put(id, result);
        json.put(id, CanonicalJson.write(result));
        return id;
    }

    @Override
    public synchronized Optional<OperationalDecisionResult> findById(String id) {
        return Optional.ofNullable(records.get(id));
    }

    @Override
    public synchronized Optional<String> findJsonById(String id) {
        return Optional.ofNullable(json.get(id));
    }

    @Override
    public synchronized List<String> listIds() {
        return new ArrayList<>(records.keySet());
    }
}
