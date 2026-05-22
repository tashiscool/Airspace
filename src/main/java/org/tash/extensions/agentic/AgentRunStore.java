package org.tash.extensions.agentic;

import java.util.List;
import java.util.Optional;

public interface AgentRunStore {
    void saveRun(AgentRunResult result);

    void saveTask(AgentTask task);

    Optional<AgentRunResult> findRun(String id);

    Optional<AgentTask> findTask(String id);

    List<AgentRunResult> runs(Integer limit);

    List<AgentRunResult> runs(AgentHistoryQuery query);

    List<AgentTask> tasks(String status, Integer limit);

    List<AgentTask> tasks(AgentHistoryQuery query);

    AgentTask transitionTask(String id, AgentTaskTransitionRequest request);

    AgentStoreStatus status();
}
