package org.tash.extensions.agentic;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.time.ZonedDateTime;

public class InMemoryAgentRunStore implements AgentRunStore {
    private final Map<String, AgentRunResult> runHistory = Collections.synchronizedMap(new LinkedHashMap<>());
    private final Map<String, AgentTask> taskHistory = Collections.synchronizedMap(new LinkedHashMap<>());

    @Override
    public void saveRun(AgentRunResult result) {
        if (result != null && result.getId() != null) {
            runHistory.put(result.getId(), result);
        }
    }

    @Override
    public void saveTask(AgentTask task) {
        if (task != null && task.getId() != null) {
            taskHistory.put(task.getId(), task);
        }
    }

    @Override
    public Optional<AgentRunResult> findRun(String id) {
        return Optional.ofNullable(id == null ? null : runHistory.get(id));
    }

    @Override
    public Optional<AgentTask> findTask(String id) {
        return Optional.ofNullable(id == null ? null : taskHistory.get(id));
    }

    @Override
    public List<AgentRunResult> runs(Integer limit) {
        AgentHistoryQuery query = new AgentHistoryQuery();
        query.setLimit(limit);
        return runs(query);
    }

    @Override
    public List<AgentRunResult> runs(AgentHistoryQuery query) {
        AgentHistoryQuery safe = query == null ? new AgentHistoryQuery() : query;
        int max = safe.getLimit() == null || safe.getLimit() <= 0 ? 25 : Math.min(safe.getLimit(), 250);
        List<AgentRunResult> values = new ArrayList<>(runHistory.values());
        values.removeIf(result -> !matchesRun(result, safe));
        Collections.reverse(values);
        return values.size() > max ? new ArrayList<>(values.subList(0, max)) : values;
    }

    @Override
    public List<AgentTask> tasks(String status, Integer limit) {
        AgentHistoryQuery query = new AgentHistoryQuery();
        query.setTaskStatus(status);
        query.setLimit(limit);
        return tasks(query);
    }

    @Override
    public List<AgentTask> tasks(AgentHistoryQuery query) {
        AgentHistoryQuery safe = query == null ? new AgentHistoryQuery() : query;
        int max = safe.getLimit() == null || safe.getLimit() <= 0 ? 50 : Math.min(safe.getLimit(), 250);
        List<AgentTask> values = new ArrayList<>();
        synchronized (taskHistory) {
            for (AgentTask task : taskHistory.values()) {
                if (matchesTask(task, safe)) {
                    values.add(task);
                }
            }
        }
        Collections.reverse(values);
        return values.size() > max ? new ArrayList<>(values.subList(0, max)) : values;
    }

    @Override
    public AgentTask transitionTask(String id, AgentTaskTransitionRequest request) {
        AgentTask known = taskHistory.get(id);
        if (known == null) {
            throw new IllegalArgumentException("Unknown agent task: " + id);
        }
        String status = request == null || request.getStatus() == null || request.getStatus().trim().isEmpty()
                ? known.getStatus()
                : request.getStatus().trim().toUpperCase(Locale.US);
        AgentTask updated = known.toBuilder()
                .status(status)
                .rationale(AgentSupport.value(known.getRationale(), "") + transitionNote(request))
                .build();
        taskHistory.put(id, updated);
        return updated;
    }

    @Override
    public AgentStoreStatus status() {
        return AgentStoreStatus.builder()
                .mode("IN_MEMORY")
                .durable(false)
                .runCount(allRunsSnapshot().size())
                .taskCount(allTasksSnapshot().size())
                .latestRunAt(latestRunAt())
                .build();
    }

    private boolean matchesRun(AgentRunResult result, AgentHistoryQuery query) {
        if (result == null) {
            return false;
        }
        return matches(result.getAgentType(), query.getAgentType(), true)
                && matches(result.getMissionId(), query.getMissionId(), false)
                && matches(result.getReservationId(), query.getReservationId(), false)
                && matches(result.getDecisionId(), query.getDecisionId(), false)
                && (query.getAccepted() == null || query.getAccepted() == result.isAccepted())
                && hasSourceFamily(result, query.getSourceFamily());
    }

    private boolean matchesTask(AgentTask task, AgentHistoryQuery query) {
        if (task == null) {
            return false;
        }
        return matches(task.getStatus(), query.getTaskStatus(), true)
                && matches(task.getPriority(), query.getTaskPriority(), true)
                && matches(task.getAssignedRole(), query.getAssignedRole(), true)
                && contains(task.getRoute(), query.getRouteContains())
                && hasSourceFamily(task.getCitations(), query.getSourceFamily());
    }

    private boolean hasSourceFamily(AgentRunResult result, String sourceFamily) {
        if (sourceFamily == null || sourceFamily.trim().isEmpty()) {
            return true;
        }
        if (hasSourceFamily(result.getCitations(), sourceFamily)) {
            return true;
        }
        for (AgentFinding finding : result.getFindings()) {
            if (hasSourceFamily(finding.getCitations(), sourceFamily)) return true;
        }
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            if (hasSourceFamily(recommendation.getCitations(), sourceFamily)) return true;
        }
        for (AgentTask task : result.getTasks()) {
            if (hasSourceFamily(task.getCitations(), sourceFamily)) return true;
        }
        for (AgentOperationalDelta delta : result.getDeltas()) {
            if (hasSourceFamily(delta.getCitations(), sourceFamily)) return true;
        }
        return result.getTraceAnswer() != null && hasSourceFamily(result.getTraceAnswer().getCitations(), sourceFamily);
    }

    private boolean hasSourceFamily(List<AgentSourceCitation> citations, String sourceFamily) {
        if (sourceFamily == null || sourceFamily.trim().isEmpty()) {
            return true;
        }
        String expected = sourceFamily.trim().toUpperCase(Locale.US);
        for (AgentSourceCitation citation : citations == null ? Collections.<AgentSourceCitation>emptyList() : citations) {
            if (citation.getSourceFamily() != null && expected.equals(citation.getSourceFamily().trim().toUpperCase(Locale.US))) {
                return true;
            }
        }
        return false;
    }

    private boolean matches(String actual, String expected, boolean normalizeCase) {
        if (expected == null || expected.trim().isEmpty()) {
            return true;
        }
        if (actual == null) {
            return false;
        }
        return normalizeCase
                ? actual.trim().equalsIgnoreCase(expected.trim())
                : actual.trim().equals(expected.trim());
    }

    private boolean contains(String actual, String expected) {
        if (expected == null || expected.trim().isEmpty()) {
            return true;
        }
        return actual != null && actual.toLowerCase(Locale.US).contains(expected.trim().toLowerCase(Locale.US));
    }

    protected List<AgentRunResult> allRunsSnapshot() {
        synchronized (runHistory) {
            return new ArrayList<>(runHistory.values());
        }
    }

    protected List<AgentTask> allTasksSnapshot() {
        synchronized (taskHistory) {
            return new ArrayList<>(taskHistory.values());
        }
    }

    private ZonedDateTime latestRunAt() {
        ZonedDateTime latest = null;
        for (AgentRunResult run : allRunsSnapshot()) {
            if (run.getGeneratedAt() != null && (latest == null || run.getGeneratedAt().isAfter(latest))) {
                latest = run.getGeneratedAt();
            }
        }
        return latest;
    }

    private String transitionNote(AgentTaskTransitionRequest request) {
        if (request == null) {
            return "";
        }
        String actor = request.getActor() == null || request.getActor().trim().isEmpty() ? "operator" : request.getActor().trim();
        String note = request.getNote() == null || request.getNote().trim().isEmpty() ? "status transition" : request.getNote().trim();
        return " Transition by " + actor + ": " + note;
    }
}
