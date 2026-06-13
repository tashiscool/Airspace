package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@ApplicationScoped
public class AgentPolicyEnforcer {
    public List<String> validate(AgentRunResult result, AgentPolicy policy) {
        AgentPolicy safePolicy = policy == null ? AgentPolicy.builder().build() : policy;
        List<String> diagnostics = new ArrayList<>();
        if (result == null) {
            return diagnostics;
        }
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            String action = recommendation.getAction() == null ? "" : recommendation.getAction().toUpperCase(Locale.US);
            if (!safePolicy.isAllowExternalSend() && (action.contains("SEND") || action.contains("TRANSMIT"))) {
                diagnostics.add("Policy blocks external-send recommendation without human approval: " + recommendation.getId());
            }
            if (!safePolicy.isAllowOfficialStateMutation()
                    && (action.contains("APPROVE") || action.contains("CANCEL") || action.contains("SUBMIT")
                    || action.contains("COMPLETE") || action.contains("MUTATE"))) {
                diagnostics.add("Policy blocks official workflow mutation recommendation: " + recommendation.getId());
            }
            if (safePolicy.isRequireHumanApprovalForDrafts() && action.contains("DRAFT")
                    && !recommendation.isHumanApprovalRequired()) {
                diagnostics.add("Policy requires human approval for draft recommendation: " + recommendation.getId());
            }
            if ((action.contains("SEND") || action.contains("TRANSMIT") || action.contains("APPROVE")
                    || action.contains("CANCEL") || action.contains("SUBMIT") || action.contains("COMPLETE")
                    || action.contains("DRAFT"))
                    && recommendation.getHumanReviewMode() != HumanReviewMode.PUSH_APPROVAL) {
                diagnostics.add("Policy requires PUSH_APPROVAL human-review mode for official/draft recommendation: " + recommendation.getId());
            }
        }
        for (AgentToolCall toolCall : result.getToolCalls()) {
            String name = toolCall.getToolName() == null ? "" : toolCall.getToolName().toUpperCase(Locale.US);
            String sideEffect = toolCall.getSideEffectLevel() == null ? "" : toolCall.getSideEffectLevel().toUpperCase(Locale.US);
            if (!safePolicy.isAllowExternalSend() && (name.contains("SEND") || name.contains("TRANSMIT"))) {
                diagnostics.add("Policy blocks external-send tool call: " + toolCall.getId());
            }
            if (!safePolicy.isAllowOfficialStateMutation()
                    && (name.contains("APPROVE") || name.contains("CANCEL") || name.contains("SUBMIT")
                    || name.contains("COMPLETE") || name.contains("MUTATE")
                    || "MUTATING".equals(sideEffect))) {
                diagnostics.add("Policy blocks official workflow mutation tool call: " + toolCall.getId());
            }
            if ("REQUIRES_APPROVAL".equals(sideEffect)
                    && !safePolicy.isAllowExternalSend()
                    && !safePolicy.isAllowOfficialStateMutation()) {
                diagnostics.add("Policy blocks approval-required MCP tool call under draft-only policy: " + toolCall.getId());
            }
        }
        return diagnostics;
    }
}
