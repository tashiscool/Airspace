package org.tash.extensions.agentic.queue;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.AgentPolicy;
import org.tash.extensions.agentic.AgentRunRequest;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgentToolCall;
import org.tash.extensions.agentic.AgenticOperationsService;
import org.tash.extensions.agentic.mcp.AgentToolPlan;
import org.tash.extensions.agentic.mcp.AgentToolPlanner;
import org.tash.extensions.agentic.mcp.CuratedMcpCatalog;
import org.tash.extensions.agentic.mcp.CuratedMcpInvocationService;
import org.tash.extensions.agentic.mcp.McpEvidenceReceipt;
import org.tash.extensions.agentic.mcp.McpInvocationStatus;
import org.tash.extensions.agentic.mcp.McpToolDescriptor;
import org.tash.extensions.agentic.mcp.McpToolInvocationRequest;
import org.tash.extensions.agentic.mcp.McpToolInvocationResult;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@ApplicationScoped
public class AgentJobConsumer {
    @Inject
    AgentJobQueueService queueService;
    @Inject
    AgentToolPlanner toolPlanner;
    @Inject
    CuratedMcpInvocationService invocationService;
    @Inject
    AgenticOperationsService agenticOperationsService;

    public AgentJobResult enqueueAndMaybeRun(AgentJobRequest request) {
        AgentJobResult queued = queue().enqueue(request);
        if (queue().isAutoConsume() && queued.getStatus() == AgentJobStatus.QUEUED) {
            return drainOnce();
        }
        return queued;
    }

    public AgentJobResult drainOnce() {
        return queue().nextPending()
                .map(this::process)
                .orElse(null);
    }

    public List<AgentJobResult> drainAll() {
        List<AgentJobResult> results = new ArrayList<>();
        AgentJobResult result;
        while ((result = drainOnce()) != null) {
            results.add(result);
        }
        return results;
    }

    private AgentJobResult process(AgentJobResult queued) {
        ZonedDateTime started = ZonedDateTime.now(ZoneOffset.UTC);
        AgentJobResult running = queued.toBuilder()
                .status(AgentJobStatus.RUNNING)
                .startedAt(started)
                .build();
        queue().save(running);
        List<String> diagnostics = new ArrayList<>(running.getDiagnostics());
        List<AgentToolCall> calls = new ArrayList<>();
        List<McpEvidenceReceipt> receipts = new ArrayList<>();
        try {
            AgentRunRequest request = running.getRequest() == null || running.getRequest().getAgentRunRequest() == null
                    ? new AgentRunRequest()
                    : running.getRequest().getAgentRunRequest();
            AgentPolicy policy = request.getPolicy() == null ? AgentPolicy.builder().build() : request.getPolicy();
            AgentToolPlan plan = planner().plan(request, policy);
            diagnostics.addAll(plan.getDiagnostics());
            for (McpToolDescriptor descriptor : plan.getAvailableTools()) {
                Map<String, Object> arguments = argumentsFor(descriptor, running.getRequest(), request);
                McpToolInvocationResult toolResult = invoker().call(McpToolInvocationRequest.builder()
                        .serverId(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY)
                        .toolId(descriptor.getId())
                        .actor(request.getActor())
                        .policy(policy)
                        .arguments(arguments)
                        .build());
                if (toolResult.getToolCall() != null) {
                    calls.add(toolResult.getToolCall());
                }
                if (toolResult.getEvidenceReceipt() != null) {
                    receipts.add(toolResult.getEvidenceReceipt());
                }
                if (toolResult.getStatus() != McpInvocationStatus.ACCEPTED) {
                    diagnostics.add("MCP tool " + descriptor.getId() + " returned " + toolResult.getStatus());
                }
            }
            request.setToolCalls(calls);
            AgentRunResult runResult = service().run(request);
            AgentJobResult completed = running.toBuilder()
                    .status(runResult.isAccepted() ? AgentJobStatus.SUCCEEDED : AgentJobStatus.DENIED)
                    .runResult(runResult)
                    .toolCalls(calls)
                    .receipts(receipts)
                    .diagnostics(diagnostics)
                    .completedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .build();
            queue().save(completed);
            return completed;
        } catch (Exception ex) {
            diagnostics.add("Agent job failed: " + ex.getClass().getSimpleName() + ": " + ex.getMessage());
            AgentJobResult failed = running.toBuilder()
                    .status(AgentJobStatus.FAILED)
                    .toolCalls(calls)
                    .receipts(receipts)
                    .diagnostics(diagnostics)
                    .completedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .build();
            queue().save(failed);
            return failed;
        }
    }

    private Map<String, Object> argumentsFor(McpToolDescriptor descriptor, AgentJobRequest jobRequest, AgentRunRequest request) {
        Map<String, Object> arguments = new LinkedHashMap<>();
        if (jobRequest != null && jobRequest.getToolArguments() != null
                && jobRequest.getToolArguments().get(descriptor.getId()) != null) {
            arguments.putAll(jobRequest.getToolArguments().get(descriptor.getId()));
        }
        putIfPresent(arguments, "missionId", request.getMissionId());
        putIfPresent(arguments, "reservationId", request.getReservationId());
        putIfPresent(arguments, "decisionId", request.getDecisionId());
        putIfPresent(arguments, "hazardOrDecisionId", request.getHazardOrDecisionId());
        putIfPresent(arguments, "actor", request.getActor());
        putIfPresent(arguments, "feedArtifactId", request.getFeedArtifactId());
        putIfPresent(arguments, "type", request.getReferenceType());
        putIfPresent(arguments, "identifier", request.getReferenceIdentifier());
        if (descriptor.getId().contains("pireps.relevant")) {
            arguments.putIfAbsent("altitudeToleranceFeet", 2000);
            arguments.putIfAbsent("recencyMinutes", 60);
            arguments.putIfAbsent("corridorNauticalMiles", 40);
        }
        return arguments;
    }

    private void putIfPresent(Map<String, Object> arguments, String key, String value) {
        if (!arguments.containsKey(key) && value != null && !value.trim().isEmpty()) {
            arguments.put(key, value);
        }
    }

    private AgentJobQueueService queue() {
        return queueService == null ? new AgentJobQueueService() : queueService;
    }

    private AgentToolPlanner planner() {
        return toolPlanner == null ? new AgentToolPlanner() : toolPlanner;
    }

    private CuratedMcpInvocationService invoker() {
        if (invocationService == null) {
            throw new IllegalStateException("CuratedMcpInvocationService is not configured");
        }
        return invocationService;
    }

    private AgenticOperationsService service() {
        if (agenticOperationsService == null) {
            throw new IllegalStateException("AgenticOperationsService is not configured");
        }
        return agenticOperationsService;
    }
}
