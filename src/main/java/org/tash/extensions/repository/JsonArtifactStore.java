package org.tash.extensions.repository;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import org.tash.extensions.engine.CanonicalJson;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonArtifactStore {
    private final Path path;
    private final ObjectMapper mapper = new ObjectMapper().registerModule(new JavaTimeModule());
    private final Map<String, JsonArtifactRecord> records = new LinkedHashMap<>();

    public JsonArtifactStore(Path path) {
        this.path = path;
        load();
    }

    public synchronized String save(String type, Object value) {
        String json = CanonicalJson.write(value);
        String id = type + ":" + CanonicalJson.sha256String(json);
        records.put(id, JsonArtifactRecord.builder()
                .id(id)
                .type(type)
                .json(json)
                .storedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build());
        persist();
        return id;
    }

    public synchronized Optional<String> findJsonById(String id) {
        JsonArtifactRecord record = records.get(id);
        return record == null ? Optional.empty() : Optional.ofNullable(record.getJson());
    }

    public synchronized List<String> listIds() {
        return new ArrayList<>(records.keySet());
    }

    private void load() {
        if (path == null || !Files.exists(path)) {
            return;
        }
        try {
            if (Files.size(path) == 0) {
                return;
            }
            List<JsonArtifactRecord> loaded = mapper.readValue(path.toFile(),
                    new TypeReference<List<JsonArtifactRecord>>() {});
            for (JsonArtifactRecord record : loaded) {
                records.put(record.getId(), record);
            }
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to load JSON artifact store " + path, ex);
        }
    }

    private void persist() {
        if (path == null) {
            return;
        }
        try {
            if (path.getParent() != null) {
                Files.createDirectories(path.getParent());
            }
            mapper.writerWithDefaultPrettyPrinter().writeValue(path.toFile(), new ArrayList<>(records.values()));
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to persist JSON artifact store " + path, ex);
        }
    }
}
