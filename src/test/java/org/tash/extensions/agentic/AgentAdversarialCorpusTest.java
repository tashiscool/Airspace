package org.tash.extensions.agentic;

import org.junit.jupiter.api.Test;

import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class AgentAdversarialCorpusTest {
    @Test
    void adversarialCorpusProducesExpectedClarificationAndCitationBehavior() {
        AgentAdversarialCorpusRunner runner = new AgentAdversarialCorpusRunner();
        for (AgentAdversarialScenario scenario : runner.load()) {
            AgentRunResult result = runner.run(scenario);
            Set<String> categories = result.getFindings().stream()
                    .map(AgentFinding::getCategory)
                    .collect(Collectors.toSet());

            for (String expected : scenario.expectedCategories) {
                assertTrue(categories.contains(expected), scenario.id + " missing " + expected + " in " + categories);
            }
            for (String forbidden : scenario.forbiddenCategories) {
                assertFalse(categories.contains(forbidden), scenario.id + " incorrectly emitted " + forbidden);
            }
            assertTrue(result.getFindings().stream().allMatch(finding -> finding.getCitations() != null && !finding.getCitations().isEmpty()),
                    scenario.id + " emitted uncited finding");
            assertTrue(result.getAssessments().stream().anyMatch(assessment -> assessment.getHumanReviewMode() == scenario.expectedReviewMode),
                    scenario.id + " missing expected HITL mode " + scenario.expectedReviewMode);
            assertTrue(result.getEvaluation().isStabilityAppliedAutomatically(), scenario.id + " missing automatic stability gate");
            assertEquals(AgentStabilityMode.WARN_ONLY.name(), result.getEvaluation().getStabilityMode());
        }
    }

    @Test
    void policyStillDeniesAdversarialOfficialSendAttempt() {
        AgentRunResult result = AgentRunResult.builder()
                .id("agent-adversarial-send")
                .agentType("COORDINATION_DRAFT")
                .accepted(true)
                .recommendations(java.util.Collections.singletonList(AgentRecommendation.builder()
                        .id("bad-send")
                        .action("SEND_EXTERNAL_MESSAGE")
                        .summary("Transmit this draft directly")
                        .humanApprovalRequired(false)
                        .citations(java.util.Collections.singletonList(AgentSupport.citation("USNS", "draft", "Draft", "/messages")))
                        .build()))
                .build();

        AgentEvaluationService service = new AgentEvaluationService();
        service.citationValidator = new AgentCitationValidator();
        service.policyEnforcer = new AgentPolicyEnforcer();
        AgentEvaluationSummary summary = service.evaluate(result, AgentPolicy.builder().build());

        assertFalse(summary.isAccepted());
        assertTrue(summary.getErrors().stream().anyMatch(error -> error.contains("external-send")));
        assertTrue(summary.getPolicyViolationCount() >= 1);
        assertTrue(new AgentAssessmentBuilder().normalize(result).getRecommendations().stream()
                .allMatch(recommendation -> recommendation.getHumanReviewMode() == HumanReviewMode.PUSH_APPROVAL));
    }
}
