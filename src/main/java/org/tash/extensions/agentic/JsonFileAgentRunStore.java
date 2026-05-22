package org.tash.extensions.agentic;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class JsonFileAgentRunStore extends InMemoryAgentRunStore {
    private final Path path;
    private final ObjectMapper mapper = new ObjectMapper().registerModule(new JavaTimeModule());
    private boolean loading;

    public JsonFileAgentRunStore(Path path) {
        this.path = path;
        load();
    }

    @Override
    public synchronized void saveRun(AgentRunResult result) {
        super.saveRun(result);
        persist();
    }

    @Override
    public synchronized void saveTask(AgentTask task) {
        super.saveTask(task);
        persist();
    }

    @Override
    public synchronized AgentTask transitionTask(String id, AgentTaskTransitionRequest request) {
        AgentTask updated = super.transitionTask(id, request);
        persist();
        return updated;
    }

    @Override
    public AgentStoreStatus status() {
        return super.status().toBuilder()
                .mode("JSON_FILE")
                .durable(true)
                .path(path == null ? null : path.toString())
                .build();
    }

    private void load() {
        if (path == null || !Files.exists(path)) {
            return;
        }
        try {
            if (Files.size(path) == 0) {
                return;
            }
            loading = true;
            AgentRunStoreState state = mapper.readValue(path.toFile(), AgentRunStoreState.class);
            for (AgentRunResult run : state.getRuns()) {
                super.saveRun(run);
            }
            for (AgentTask task : state.getTasks()) {
                super.saveTask(task);
            }
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to load agent run store " + path, ex);
        } finally {
            loading = false;
        }
    }

    private void persist() {
        if (loading || path == null) {
            return;
        }
        try {
            if (path.getParent() != null) {
                Files.createDirectories(path.getParent());
            }
            AgentRunStoreState state = new AgentRunStoreState();
            state.setRuns(allRunsSnapshot());
            state.setTasks(allTasksSnapshot());
            mapper.writerWithDefaultPrettyPrinter().writeValue(path.toFile(), state);
        } catch (IOException ex) {
            throw new IllegalStateException("Unable to persist agent run store " + path, ex);
        }
    }

    @Data
    public static class AgentRunStoreState {
        private List<AgentRunResult> runs = new ArrayList<>();
        private List<AgentTask> tasks = new ArrayList<>();
    }
}
