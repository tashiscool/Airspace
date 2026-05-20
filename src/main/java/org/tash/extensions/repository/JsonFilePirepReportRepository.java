package org.tash.extensions.repository;

import org.tash.extensions.weather.pirep.PirepIngestResult;

import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonFilePirepReportRepository implements PirepReportRepository {
    private final JsonArtifactStore store;
    private final Map<String, PirepIngestResult> currentProcessRecords = new LinkedHashMap<>();

    public JsonFilePirepReportRepository(Path path) {
        this.store = new JsonArtifactStore(path);
    }

    @Override
    public synchronized String save(PirepIngestResult result) {
        String id = store.save("pirep", result);
        currentProcessRecords.put(id, result);
        return id;
    }

    @Override
    public synchronized Optional<PirepIngestResult> findById(String id) {
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
