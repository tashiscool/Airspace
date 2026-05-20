package org.tash.extensions.repository;

import org.tash.extensions.engine.OperationalDecisionResult;

import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonFileOperationalDecisionRepository implements OperationalDecisionRepository {
    private final JsonArtifactStore store;
    private final Map<String, OperationalDecisionResult> currentProcessRecords = new LinkedHashMap<>();

    public JsonFileOperationalDecisionRepository(Path path) {
        this.store = new JsonArtifactStore(path);
    }

    @Override
    public synchronized String save(OperationalDecisionResult result) {
        String id = store.save("decision", result);
        currentProcessRecords.put(id, result);
        return id;
    }

    @Override
    public synchronized Optional<OperationalDecisionResult> findById(String id) {
        return Optional.ofNullable(currentProcessRecords.get(id));
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
