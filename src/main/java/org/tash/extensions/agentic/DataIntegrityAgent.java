package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@ApplicationScoped
public class DataIntegrityAgent {
    @Inject
    AirspaceProductService productService;

    public AgentRunResult scan(AgentRunRequest request) {
        String missionId = missionId(request);
        ProductDtos.MissionWeatherVerdictSummary verdict = productService.missionWeatherVerdict(missionId);
        ProductDtos.RouteImpactSummary impact = productService.routeImpact(missionId, request == null ? null : request.getReservationId());
        List<AgentFinding> findings = new ArrayList<>();
        List<ProductDtos.MessageSummary> messages = productService.mission(missionId).getMessages();
        for (String diagnostic : impact.getDiagnostics()) {
            findings.add(finding("MALFORMED_OR_UNSUPPORTED_INPUT", "MEDIUM", diagnostic, AgentSupport.citations(impact)));
        }
        if (verdict.isStale()) {
            findings.add(finding("STALE_PRODUCT", "MEDIUM", "One or more mission weather/PIREP products are stale.", sourceCitations(verdict)));
        }
        for (ProductDtos.WeatherSourceSummary source : verdict.getSources()) {
            String text = ((source.getLabel() == null ? "" : source.getLabel()) + " " + (source.getRationale() == null ? "" : source.getRationale())).toUpperCase(Locale.US);
            if ((text.contains("SIGMET") || text.contains("AIRMET") || text.contains("NOTAM")) && !text.matches(".*(BOUNDED|FROM|WI |WITHIN|AREA|RADIUS|LAT|LONG|N[0-9]{4}|W[0-9]{5}).*")) {
                findings.add(finding("MISSING_GEOMETRY", "LOW",
                        source.getFamily() + " " + source.getId() + " may need geometry enrichment for map and route-impact review.",
                        java.util.Collections.singletonList(AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather"))));
            }
        }
        if (impact.getCandidateComparisons() != null) {
            impact.getCandidateComparisons().stream()
                    .filter(candidate -> candidate.getResidualConstraints() != null && !candidate.getResidualConstraints().isEmpty())
                    .forEach(candidate -> findings.add(finding("RESIDUAL_RISK", "MEDIUM",
                            candidate.getLabel() + " still has " + candidate.getResidualConstraints().size() + " residual constraint(s).",
                            AgentSupport.citations(impact))));
            impact.getCandidateComparisons().stream()
                    .filter(candidate -> candidate.getAvoidedConstraints() != null && !candidate.getAvoidedConstraints().isEmpty())
                    .filter(candidate -> candidate.getResidualConstraints() != null && candidate.getResidualConstraints().stream()
                            .anyMatch(constraint -> containsAny(constraint.getFamily() + " " + constraint.getLabel() + " " + constraint.getSourceRef(), "NOTAM", "CARF", "ALTRV")))
                    .forEach(candidate -> findings.add(finding("REROUTE_RESIDUAL_CONSTRAINT_CONFLICT", "HIGH",
                            candidate.getLabel() + " avoids at least one hazard but still retains NOTAM/CARF/ALTRV residual constraints.",
                            AgentSupport.citations(impact))));
        }
        findings.addAll(contradictionFindings(messages, impact));
        findings.addAll(pirepRelevanceFindings(missionId, request, impact));
        findings.addAll(sourceReferenceFindings(impact));
        if (findings.isEmpty()) {
            findings.add(finding("DATA_QUALITY", "INFO", "No stale, malformed, missing-geometry, or residual-risk findings were detected for this mission.", AgentSupport.citations(impact)));
        }
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "data-integrity:" + missionId + ":" + findings.size()))
                .agentType("DATA_INTEGRITY")
                .summary(findings.size() + " data quality finding(s) for mission " + missionId + ".")
                .confidence(0.82)
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(findings)
                .citations(findings.get(0).getCitations())
                .build();
    }

    private AgentFinding finding(String category, String severity, String message, List<AgentSourceCitation> citations) {
        return AgentFinding.builder()
                .id(AgentSupport.id("finding", category + ":" + message))
                .category(category)
                .severity(severity)
                .message(message)
                .confidence("INFO".equals(severity) ? 0.9 : 0.74)
                .citations(citations)
                .build();
    }

    private List<AgentSourceCitation> sourceCitations(ProductDtos.MissionWeatherVerdictSummary verdict) {
        List<AgentSourceCitation> citations = new ArrayList<>();
        verdict.getSources().forEach(source -> citations.add(AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather")));
        return citations.isEmpty() ? java.util.Collections.singletonList(AgentSupport.citation("ENGINE", verdict.getMissionId(), "Mission weather verdict", "/missions/" + verdict.getMissionId())) : citations;
    }

    private String missionId(AgentRunRequest request) {
        if (request != null && request.getMissionId() != null && !request.getMissionId().trim().isEmpty()) {
            return request.getMissionId();
        }
        return productService.missions().get(0).getId();
    }

    private List<AgentFinding> contradictionFindings(List<ProductDtos.MessageSummary> messages,
                                                     ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        boolean severeSigmetOrAirmet = messages.stream().anyMatch(message ->
                containsAny(message.getFamily() + " " + message.getSubject() + " " + message.getRawText(),
                        "SIGMET", "AIRMET")
                        && containsAny(message.getRawText(), "SEV", "SEVERE", "EXTREME", "EMBD TS", "ICING", "TURB"));
        boolean smoothOrNegativePirep = messages.stream().anyMatch(message ->
                containsAny(message.getFamily() + " " + message.getRawText(), "PIREP", "/TB", "/IC")
                        && containsAny(message.getRawText(), "NEG", "SMOOTH", "NO ICE", "NIL IC", "/TB SMOOTH", "/IC NEG"));
        if (severeSigmetOrAirmet && smoothOrNegativePirep) {
            findings.add(finding("CONTRADICTORY_PIREP_WEATHER", "MEDIUM",
                    "Severe SIGMET/AIRMET-style hazard and nearby smooth/negative PIREP text are both present; operator should review source timing, altitude, and proximity.",
                    citations(messages, "WEATHER", "PIREP", impact)));
        }
        boolean staleMetarTaf = messages.stream().anyMatch(message ->
                containsAny(message.getFamily(), "METAR", "TAF")
                        && containsAny(message.getRawText(), "AUTO", "COR", "AMD", "TAF", "METAR")
                        && message.getCreatedAt() != null
                        && message.getCreatedAt().isBefore(java.time.ZonedDateTime.now(java.time.ZoneOffset.UTC).minusHours(6)));
        boolean fresherHazard = messages.stream().anyMatch(message ->
                containsAny(message.getFamily(), "SIGMET", "AIRMET", "CWAP", "CWAF", "PIREP")
                        && message.getCreatedAt() != null
                        && message.getCreatedAt().isAfter(java.time.ZonedDateTime.now(java.time.ZoneOffset.UTC).minusHours(2)));
        if (staleMetarTaf && fresherHazard) {
            findings.add(finding("STALE_TERMINAL_WEATHER_CONFLICT", "MEDIUM",
                    "Stale METAR/TAF text coexists with fresher hazard traffic; terminal weather should not suppress newer route-impact guidance.",
                    citations(messages, "METAR", "TAF", impact)));
        }
        return findings;
    }

    private List<AgentFinding> pirepRelevanceFindings(String missionId,
                                                      AgentRunRequest request,
                                                      ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        ProductDtos.PirepRelevanceRequest pirepRequest = new ProductDtos.PirepRelevanceRequest();
        pirepRequest.setReservationId(request == null ? null : request.getReservationId());
        ProductDtos.PirepRelevanceResult relevance = productService.relevantPireps(missionId, pirepRequest);
        if (!relevance.getExcluded().isEmpty() && relevance.getRelevant().isEmpty() && impact.getAction() != null
                && containsAny(impact.getAction(), "BLOCK", "REROUTE", "AVOID")) {
            findings.add(finding("PIREP_RELEVANCE_MISMATCH", "MEDIUM",
                    "PIREP traffic exists but falls outside the selected route/altitude/recency filter; it should not be treated as a direct blockage driver without review.",
                    AgentSupport.citations(impact)));
        }
        return findings;
    }

    private List<AgentFinding> sourceReferenceFindings(ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        if ((impact.getSourceRefs() == null || impact.getSourceRefs().isEmpty())
                && ("BLOCKED".equalsIgnoreCase(impact.getAction()) || "REROUTE".equalsIgnoreCase(impact.getAction()) || "AVOID".equalsIgnoreCase(impact.getAction()))) {
            findings.add(finding("MISSING_SOURCE_REFS", "HIGH",
                    "Route-impact action requires exact source refs but none were attached.",
                    AgentSupport.citations(impact)));
        }
        if (impact.getCandidateComparisons() != null) {
            impact.getCandidateComparisons().stream()
                    .filter(candidate -> candidate.getSourceRefs() == null || candidate.getSourceRefs().isEmpty())
                    .forEach(candidate -> findings.add(finding("MISSING_ROUTE_CANDIDATE_SOURCE_REFS", "MEDIUM",
                            candidate.getLabel() + " has no candidate-level source refs.",
                            AgentSupport.citations(impact))));
        }
        return findings;
    }

    private List<AgentSourceCitation> citations(List<ProductDtos.MessageSummary> messages,
                                                String familyA,
                                                String familyB,
                                                ProductDtos.RouteImpactSummary impact) {
        List<AgentSourceCitation> citations = new ArrayList<>();
        messages.stream()
                .filter(message -> containsAny(message.getFamily() + " " + message.getRawText(), familyA, familyB))
                .limit(6)
                .forEach(message -> citations.add(AgentSupport.citation(message.getFamily(), message.getId(), message.getSubject(), "/messages/" + message.getId())));
        if (citations.isEmpty()) {
            citations.addAll(AgentSupport.citations(impact));
        }
        return citations;
    }

    private boolean containsAny(String value, String... tokens) {
        String text = value == null ? "" : value.toUpperCase(Locale.US);
        for (String token : tokens) {
            if (text.contains(token.toUpperCase(Locale.US))) {
                return true;
            }
        }
        return false;
    }
}
