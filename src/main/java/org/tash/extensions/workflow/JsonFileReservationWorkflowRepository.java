package org.tash.extensions.workflow;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonFileReservationWorkflowRepository implements ReservationWorkflowRepository {
    private final Path path;
    private final ObjectMapper mapper;
    private final Map<String, ReservationWorkflowRecord> records = new LinkedHashMap<>();

    public JsonFileReservationWorkflowRepository(Path path) {
        this.path = path;
        this.mapper = new ObjectMapper().registerModule(new JavaTimeModule());
        load();
    }

    @Override
    public synchronized ReservationWorkflowRecord save(ReservationWorkflowRecord record) {
        records.put(record.getId(), record);
        persist();
        return record;
    }

    @Override
    public synchronized Optional<ReservationWorkflowRecord> findById(String id) {
        return Optional.ofNullable(records.get(id));
    }

    @Override
    public synchronized List<ReservationWorkflowRecord> findOpen() {
        List<ReservationWorkflowRecord> open = new ArrayList<>();
        for (ReservationWorkflowRecord record : records.values()) {
            if (record.getState() != ReservationWorkflowState.CANCELLED
                    && record.getState() != ReservationWorkflowState.COMPLETED
                    && record.getState() != ReservationWorkflowState.REJECTED) {
                open.add(record);
            }
        }
        return open;
    }

    @Override
    public synchronized ReservationWorkflowRecord appendAuditEvent(String id, ReservationWorkflowAuditEvent event) {
        ReservationWorkflowRecord record = records.get(id);
        if (record == null) {
            throw new IllegalArgumentException("Unknown reservation workflow record: " + id);
        }
        record.getAuditEvents().add(event);
        return save(record);
    }

    private void load() {
        if (!Files.exists(path)) {
            return;
        }
        try {
            if (Files.size(path) == 0) {
                return;
            }
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to inspect reservation workflow repository " + path, ex);
        }
        try {
            List<ReservationWorkflowRecord> stored = mapper.readValue(path.toFile(),
                    new TypeReference<List<ReservationWorkflowRecord>>() {});
            for (ReservationWorkflowRecord record : stored) {
                records.put(record.getId(), record);
            }
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to load reservation workflow repository " + path, ex);
        }
    }

    private void persist() {
        try {
            if (path.getParent() != null) {
                Files.createDirectories(path.getParent());
            }
            mapper.writerWithDefaultPrettyPrinter().writeValue(path.toFile(), new ArrayList<>(records.values()));
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to persist reservation workflow repository " + path, ex);
        }
    }
}
