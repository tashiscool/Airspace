package org.tash.extensions.product.api;

import jakarta.inject.Inject;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.QueryParam;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.agentic.AgentTask;
import org.tash.extensions.agentic.AgentTaskTransitionRequest;
import org.tash.extensions.agentic.AgentHistoryQuery;
import org.tash.extensions.agentic.AgentRunRequest;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgentStoreStatus;
import org.tash.extensions.agentic.AgenticRiskAssessment;
import org.tash.extensions.agentic.AgenticRiskChecklistService;
import org.tash.extensions.agentic.AgenticOperationsService;
import org.tash.extensions.agentic.AgentOperationalDelta;
import org.tash.extensions.agentic.OperationalDeltaService;
import org.tash.extensions.agentic.ScenarioFixtureBundle;
import org.tash.extensions.agentic.ScenarioFixtureGenerator;
import org.tash.extensions.agentic.ScenarioFixtureRequest;
import org.tash.extensions.agentic.mcp.CuratedMcpInvocationService;
import org.tash.extensions.agentic.mcp.McpEvidenceReceipt;
import org.tash.extensions.agentic.mcp.McpServerDefinition;
import org.tash.extensions.agentic.mcp.McpToolDescriptor;
import org.tash.extensions.agentic.mcp.McpToolInvocationRequest;
import org.tash.extensions.agentic.mcp.McpToolInvocationResult;
import org.tash.extensions.agentic.mcp.McpToolRegistry;
import org.tash.extensions.agentic.queue.AgentJobConsumer;
import org.tash.extensions.agentic.queue.AgentJobQueueService;
import org.tash.extensions.agentic.queue.AgentJobRequest;
import org.tash.extensions.agentic.queue.AgentJobResult;
import org.tash.extensions.agentic.stability.AgentStabilityHarness;
import org.tash.extensions.agentic.stability.AgentStabilityRequest;
import org.tash.extensions.agentic.stability.AgentStabilityResult;

import java.util.List;
import java.util.Map;

@Path("/api/agents")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class AgentResource {
    @Inject
    AgenticOperationsService agenticOperationsService;
    @Inject
    OperationalDeltaService operationalDeltaService;
    @Inject
    ScenarioFixtureGenerator scenarioFixtureGenerator;
    @Inject
    McpToolRegistry mcpToolRegistry;
    @Inject
    CuratedMcpInvocationService mcpInvocationService;
    @Inject
    AgentJobQueueService agentJobQueueService;
    @Inject
    AgentJobConsumer agentJobConsumer;
    @Inject
    AgentStabilityHarness agentStabilityHarness;
    @Inject
    AgenticRiskChecklistService agenticRiskChecklistService;

    @POST
    @Path("/run")
    public AgentRunResult run(AgentRunRequest request) {
        return agenticOperationsService.run(request);
    }

    @POST
    @Path("/weather-impact")
    public AgentRunResult weatherImpact(AgentRunRequest request) {
        return agenticOperationsService.weatherImpact(request);
    }

    @POST
    @Path("/mission-risk")
    public AgentRunResult missionRisk(AgentRunRequest request) {
        return agenticOperationsService.missionRisk(request);
    }

    @POST
    @Path("/reroute-analysis")
    public AgentRunResult rerouteAnalysis(AgentRunRequest request) {
        return agenticOperationsService.rerouteAnalysis(request);
    }

    @POST
    @Path("/coordination-draft")
    public AgentRunResult coordinationDraft(AgentRunRequest request) {
        return agenticOperationsService.coordinationDraft(request);
    }

    @POST
    @Path("/pilot-brief")
    public AgentRunResult pilotBrief(AgentRunRequest request) {
        return agenticOperationsService.pilotBrief(request);
    }

    @POST
    @Path("/data-integrity")
    public AgentRunResult dataIntegrity(AgentRunRequest request) {
        return agenticOperationsService.dataIntegrity(request);
    }

    @POST
    @Path("/replay-audit")
    public AgentRunResult replayAudit(AgentRunRequest request) {
        return agenticOperationsService.replayAudit(request);
    }

    @POST
    @Path("/delta")
    public List<AgentOperationalDelta> delta(AgentRunRequest request) {
        AgentRunRequest safe = request == null ? new AgentRunRequest() : request;
        return operationalDeltaService.compare(safe.getPreviousDecisionId(), safe.getDecisionId(), safe.getMissionId());
    }

    @POST
    @Path("/scenario/generate")
    public ScenarioFixtureBundle scenario(ScenarioFixtureRequest request) {
        return scenarioFixtureGenerator.generate(request);
    }

    @GET
    @Path("/mcp/servers")
    public List<McpServerDefinition> mcpServers() {
        return mcpToolRegistry.servers();
    }

    @GET
    @Path("/mcp/servers/{serverId}/tools")
    public List<McpToolDescriptor> mcpTools(@PathParam("serverId") String serverId) {
        return mcpToolRegistry.tools(serverId);
    }

    @POST
    @Path("/mcp/tools/call")
    public McpToolInvocationResult callMcpTool(McpToolInvocationRequest request) {
        return mcpInvocationService.call(request);
    }

    @GET
    @Path("/mcp/receipts")
    public List<McpEvidenceReceipt> mcpReceipts(@QueryParam("limit") Integer limit) {
        return mcpInvocationService.receipts(limit);
    }

    @POST
    @Path("/evaluate/stability")
    public AgentStabilityResult evaluateStability(AgentStabilityRequest request) {
        return agentStabilityHarness.evaluate(request);
    }

    @GET
    @Path("/risk-assessments")
    public List<AgenticRiskAssessment> riskAssessments() {
        return agenticRiskChecklistService.assessments();
    }

    @POST
    @Path("/jobs")
    public AgentJobResult enqueueJob(AgentJobRequest request) {
        return agentJobConsumer.enqueueAndMaybeRun(request);
    }

    @GET
    @Path("/jobs")
    public List<AgentJobResult> jobs(@QueryParam("limit") Integer limit) {
        return agentJobQueueService.jobs(limit);
    }

    @GET
    @Path("/jobs/{id}")
    public AgentJobResult job(@PathParam("id") String id) {
        return agentJobQueueService.job(id).orElseThrow(() -> new IllegalArgumentException("Unknown agent job: " + id));
    }

    @GET
    @Path("/status")
    public AgentStoreStatus status() {
        return agenticOperationsService.status();
    }

    @GET
    @Path("/metrics")
    public Map<String, Double> metrics() {
        return agenticOperationsService.metrics();
    }

    @GET
    @Path("/runs")
    public List<AgentRunResult> runs(@QueryParam("limit") Integer limit,
                                     @QueryParam("agentType") String agentType,
                                     @QueryParam("missionId") String missionId,
                                     @QueryParam("reservationId") String reservationId,
                                     @QueryParam("decisionId") String decisionId,
                                     @QueryParam("accepted") Boolean accepted,
                                     @QueryParam("sourceFamily") String sourceFamily) {
        AgentHistoryQuery query = new AgentHistoryQuery();
        query.setLimit(limit);
        query.setAgentType(agentType);
        query.setMissionId(missionId);
        query.setReservationId(reservationId);
        query.setDecisionId(decisionId);
        query.setAccepted(accepted);
        query.setSourceFamily(sourceFamily);
        return agenticOperationsService.runs(query);
    }

    @GET
    @Path("/runs/{id}")
    public AgentRunResult runById(@PathParam("id") String id) {
        return agenticOperationsService.runById(id);
    }

    @GET
    @Path("/tasks")
    public List<AgentTask> tasks(@QueryParam("status") String status,
                                 @QueryParam("limit") Integer limit,
                                 @QueryParam("priority") String priority,
                                 @QueryParam("assignedRole") String assignedRole,
                                 @QueryParam("sourceFamily") String sourceFamily,
                                 @QueryParam("routeContains") String routeContains) {
        AgentHistoryQuery query = new AgentHistoryQuery();
        query.setTaskStatus(status);
        query.setLimit(limit);
        query.setTaskPriority(priority);
        query.setAssignedRole(assignedRole);
        query.setSourceFamily(sourceFamily);
        query.setRouteContains(routeContains);
        return agenticOperationsService.tasks(query);
    }

    @GET
    @Path("/tasks/{id}")
    public AgentTask taskById(@PathParam("id") String id) {
        return agenticOperationsService.taskById(id);
    }

    @POST
    @Path("/tasks/{id}/transition")
    public AgentTask transitionTask(@PathParam("id") String id, AgentTaskTransitionRequest request) {
        return agenticOperationsService.transitionTask(id, request);
    }
}
