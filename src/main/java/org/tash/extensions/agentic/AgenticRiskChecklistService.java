package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.mcp.CuratedMcpCatalog;
import org.tash.extensions.agentic.mcp.McpToolDescriptor;
import org.tash.extensions.agentic.mcp.McpToolRegistry;

import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class AgenticRiskChecklistService {
    @Inject
    McpToolRegistry mcpToolRegistry;
    @Inject
    AgentRiskCatalog agentRiskCatalog;

    public AgenticRiskChecklistService() {
    }

    public AgenticRiskChecklistService(McpToolRegistry mcpToolRegistry) {
        this.mcpToolRegistry = mcpToolRegistry;
    }

    public List<AgenticRiskAssessment> assessments() {
        return assessments("ALL");
    }

    public List<AgenticRiskAssessment> assessments(String subjectType) {
        String normalized = subjectType == null || subjectType.trim().isEmpty()
                ? "ALL"
                : subjectType.trim().toUpperCase(java.util.Locale.US);
        McpToolRegistry registry = mcpToolRegistry == null ? new CuratedMcpCatalog() : mcpToolRegistry;
        List<AgenticRiskAssessment> assessments = new ArrayList<>();
        if ("ALL".equals(normalized) || "AGENT".equals(normalized)) {
            for (AgentDefinition agent : agents().agents()) {
                assessments.add(assessment(agent));
            }
        }
        if ("ALL".equals(normalized) || "MCP_TOOL".equals(normalized)) {
            for (McpToolDescriptor tool : registry.tools(CuratedMcpCatalog.AIRSPACE_FIRST_PARTY)) {
                assessments.add(assessment(tool));
            }
            for (McpToolDescriptor tool : registry.tools(CuratedMcpCatalog.LOCAL_MOCK_MCP)) {
                assessments.add(assessment(tool));
            }
            for (McpToolDescriptor tool : registry.tools(CuratedMcpCatalog.EXTERNAL_MCP)) {
                assessments.add(assessment(tool));
            }
        }
        return assessments;
    }

    private AgenticRiskAssessment assessment(AgentDefinition agent) {
        List<String> diagnostics = new ArrayList<>();
        if (agent.getRiskProfile() == null) {
            diagnostics.add("Agent lacks explicit agentic risk profile.");
        }
        return AgenticRiskAssessment.builder()
                .id("risk-agent-" + agent.getAgentType())
                .subjectType("AGENT")
                .subjectId(agent.getAgentType())
                .summary(agent.getName() + " risk checklist")
                .riskProfile(agent.getRiskProfile())
                .diagnostics(diagnostics)
                .build();
    }

    private AgenticRiskAssessment assessment(McpToolDescriptor tool) {
        AgenticRiskProfile profile = tool.getRiskProfile() == null
                ? AgenticRiskProfile.builder()
                .autonomyScope("Unknown until catalog metadata is completed.")
                .toolSurface(tool.getId())
                .worstCaseBlastRadius("Unknown")
                .permissionScope("Unknown")
                .dataEgress(tool.isExternal() ? "Potential external egress" : "Local Airspace service only")
                .requiredHumanReviewMode(HumanReviewMode.REVIEW_ONLY)
                .build()
                : tool.getRiskProfile();
        List<String> diagnostics = new ArrayList<>();
        if (tool.getRiskProfile() == null) {
            diagnostics.add("Tool lacks explicit agentic risk profile.");
        }
        if (tool.isExternal() || tool.isSetupRequired()) {
            diagnostics.add("Tool is external or setup-required and must remain blocked until configured and approved.");
        }
        return AgenticRiskAssessment.builder()
                .id("risk-" + tool.getId())
                .subjectType("MCP_TOOL")
                .subjectId(tool.getId())
                .summary(tool.getName() + " risk checklist")
                .riskProfile(profile)
                .diagnostics(diagnostics)
                .build();
    }

    private AgentRiskCatalog agents() {
        if (agentRiskCatalog == null) {
            agentRiskCatalog = new AgentRiskCatalog();
        }
        return agentRiskCatalog;
    }
}
