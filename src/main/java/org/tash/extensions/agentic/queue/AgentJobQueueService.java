package org.tash.extensions.agentic.queue;

import jakarta.enterprise.context.ApplicationScoped;
import org.eclipse.microprofile.config.inject.ConfigProperty;
import org.tash.extensions.agentic.AgentRunRequest;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.UUID;

@ApplicationScoped
public class AgentJobQueueService {
    @ConfigProperty(name = "airspace.agentic.queue.max-size", defaultValue = "100")
    int maxSize;
    @ConfigProperty(name = "airspace.agentic.queue.auto-consume", defaultValue = "true")
    boolean autoConsume;

    private final Deque<String> pending = new ArrayDeque<>();
    private final Map<String, AgentJobResult> jobs = Collections.synchronizedMap(new LinkedHashMap<>());

    public synchronized AgentJobResult enqueue(AgentJobRequest request) {
        int effectiveMax = maxSize <= 0 ? 100 : maxSize;
        if (pending.size() >= effectiveMax) {
            AgentJobResult denied = baseResult(request, AgentJobStatus.DENIED)
                    .diagnostics(Collections.singletonList("Agent job queue is full"))
                    .completedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .build();
            jobs.put(denied.getId(), denied);
            return denied;
        }
        AgentJobResult result = baseResult(request, AgentJobStatus.QUEUED).build();
        jobs.put(result.getId(), result);
        pending.addLast(result.getId());
        return result;
    }

    public synchronized Optional<AgentJobResult> nextPending() {
        String id = pending.pollFirst();
        return Optional.ofNullable(id == null ? null : jobs.get(id));
    }

    public synchronized void save(AgentJobResult result) {
        if (result != null && result.getId() != null) {
            jobs.put(result.getId(), result);
        }
    }

    public Optional<AgentJobResult> job(String id) {
        return Optional.ofNullable(id == null ? null : jobs.get(id));
    }

    public List<AgentJobResult> jobs(Integer limit) {
        int max = limit == null || limit <= 0 ? 25 : Math.min(limit, 250);
        List<AgentJobResult> values;
        synchronized (jobs) {
            values = new ArrayList<>(jobs.values());
        }
        Collections.reverse(values);
        return values.size() > max ? new ArrayList<>(values.subList(0, max)) : values;
    }

    public boolean isAutoConsume() {
        return autoConsume;
    }

    private AgentJobResult.AgentJobResultBuilder baseResult(AgentJobRequest request, AgentJobStatus status) {
        AgentJobRequest safe = request == null ? new AgentJobRequest() : request;
        AgentRunRequest runRequest = safe.getAgentRunRequest() == null ? new AgentRunRequest() : safe.getAgentRunRequest();
        String seed = safe.getIdempotencyKey() == null || safe.getIdempotencyKey().trim().isEmpty()
                ? UUID.randomUUID().toString()
                : safe.getIdempotencyKey() + ":" + runRequest.getAgentType() + ":" + runRequest.getMissionId() + ":" + runRequest.getDecisionId();
        String id = "agent-job-" + UUID.nameUUIDFromBytes(seed.getBytes(java.nio.charset.StandardCharsets.UTF_8));
        return AgentJobResult.builder()
                .id(id)
                .status(status)
                .request(safe)
                .createdAt(ZonedDateTime.now(ZoneOffset.UTC));
    }
}
