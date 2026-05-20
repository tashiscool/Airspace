package org.tash.extensions.weather;

import org.tash.extensions.engine.DecisionRuleCatalog;
import org.tash.extensions.engine.DecisionTraceStep;
import org.tash.extensions.engine.OperationalDecisionEngine;
import org.tash.extensions.engine.OperationalDecisionRequest;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductType;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class ScenarioCorpusRunner {
    private final WeatherProductParser parser = new WeatherProductParser();

    public List<WeatherProductParseResult> parseLineCorpus(String resource, WeatherProductType hint) throws IOException {
        List<WeatherProductParseResult> results = new ArrayList<>();
        for (String line : lines(resource)) {
            if (line.trim().isEmpty() || line.startsWith("#")) {
                continue;
            }
            WeatherProductParseResult result = parser.parse(line, hint);
            results.add(result);
            assertTrue(result.isAccepted() || result.isClassifiedOnly(),
                    "Corpus line should parse or be explicitly retained: " + line + " errors=" + result.getErrors());
        }
        return results;
    }

    public OperationalDecisionResult runEngineCorpus(String resource, ZonedDateTime decisionTime) throws IOException {
        OperationalDecisionResult result = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(decisionTime)
                .rawUsnsMessages(Collections.singletonList(resourceText(resource)))
                .build());
        for (DecisionTraceStep step : result.getTrace().getSteps()) {
            if (step.getRule() != null) {
                assertNotNull(DecisionRuleCatalog.byId(step.getRule().getId()), "Unknown trace rule " + step.getRule().getId());
            }
        }
        return result;
    }

    private List<String> lines(String resource) throws IOException {
        String text = resourceText(resource);
        List<String> lines = new ArrayList<>();
        Collections.addAll(lines, text.split("\\R"));
        return lines;
    }

    private String resourceText(String resource) throws IOException {
        try (InputStream stream = ScenarioCorpusRunner.class.getResourceAsStream(resource)) {
            assertNotNull(stream, resource);
            return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
        }
    }
}
