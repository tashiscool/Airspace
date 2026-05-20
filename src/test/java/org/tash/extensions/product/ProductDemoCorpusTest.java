package org.tash.extensions.product;

import org.junit.jupiter.api.Test;
import org.tash.extensions.feed.OperationalFeedBatchResult;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.nio.charset.StandardCharsets;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class ProductDemoCorpusTest {
    @Test
    void productDemoCorpusFlowsThroughFeedAndDecisionWithoutExternalDependencies() throws Exception {
        AirspaceProductService service = new AirspaceProductService(
                new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        String usns = resource("/scenarios/product-demo/usns-mixed-weather.txt");
        String carf = resource("/scenarios/product-demo/carf-altrv.txt");

        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("product-demo");
        feed.setType("USNS");
        feed.setRawPayload(usns);
        OperationalFeedBatchResult batch = service.ingestFeed(feed);

        assertFalse(batch.getResults().isEmpty());
        assertFalse(service.feedArtifacts().isEmpty());
        assertFalse(service.feedTransactions(batch.getResults().get(0).getEnvelope().getId()).isEmpty());

        ProductDtos.DecisionEvaluateRequest decision = new ProductDtos.DecisionEvaluateRequest();
        decision.setDecisionTime("2026-05-20T00:00:00Z");
        decision.setRawUsnsMessages(Collections.singletonList(usns));
        decision.setRawCarfMessages(Collections.singletonList(carf));
        ProductDtos.DecisionSummary summary = service.evaluateDecision(decision);

        assertNotNull(summary.getId());
        assertNotNull(summary.getAction());
        assertNotNull(service.decisionResult(summary.getId()).getReplayBundle());
        assertTrue(service.metrics().get("product.feedArtifacts") >= 1.0);
        assertTrue(service.metrics().get("product.decisions") >= 1.0);
    }

    private String resource(String path) throws Exception {
        return new String(getClass().getResourceAsStream(path).readAllBytes(), StandardCharsets.UTF_8);
    }
}
