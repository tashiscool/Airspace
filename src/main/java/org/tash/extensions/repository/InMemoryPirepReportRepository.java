package org.tash.extensions.repository;

import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.weather.pirep.PirepIngestResult;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class InMemoryPirepReportRepository implements PirepReportRepository {
    private final Map<String, PirepIngestResult> records = new LinkedHashMap<>();
    private final Map<String, String> json = new LinkedHashMap<>();

    @Override
    public synchronized String save(PirepIngestResult result) {
        String reportId = result == null || result.getReport() == null ? null : result.getReport().getId();
        String id = reportId == null ? "pirep:" + CanonicalJson.sha256(result) : "pirep:" + reportId;
        records.put(id, result);
        json.put(id, CanonicalJson.write(result));
        return id;
    }

    @Override
    public synchronized Optional<PirepIngestResult> findById(String id) {
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
