package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.carf.altrv.AltrvParser;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.NotamAirspaceParser;
import org.tash.extensions.notam.NotamFieldParseResult;
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
    private final AltrvParser altrvParser = new AltrvParser();
    private final DomesticNotamParser domesticNotamParser = new DomesticNotamParser();
    private final NotamAirspaceParser notamAirspaceParser = new NotamAirspaceParser();

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
        findings.addAll(altrvGrammarFindings(messages, impact));
        findings.addAll(domesticNotamSemanticFindings(messages, impact));
        findings.addAll(icaoNotamFieldFindings(messages, impact));
        findings.addAll(serviceCommandFindings(impact));
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

    private List<AgentFinding> domesticNotamSemanticFindings(List<ProductDtos.MessageSummary> messages,
                                                             ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        for (ProductDtos.MessageSummary message : messages) {
            String raw = message.getRawText() == null ? "" : message.getRawText();
            if (!looksLikeDomesticNotam(message, raw)) {
                continue;
            }
            try {
                DomesticNotamParseResult parsed = domesticNotamParser.parseDetailed(raw);
                List<AgentSourceCitation> citations = java.util.Collections.singletonList(AgentSupport.citation(
                        value(message.getFamily(), "DOMESTIC_NOTAM"),
                        value(message.getId(), "domestic-notam"),
                        value(message.getSubject(), "Domestic NOTAM"),
                        message.getId() == null ? "/notams" : "/messages/" + message.getId()));
                if (!parsed.isAccepted()) {
                    findings.add(finding("MALFORMED_DOMESTIC_NOTAM", "MEDIUM",
                            "Domestic NOTAM record did not match the DOM1 record-shape oracle: "
                                    + value(parsed.getRejectionReason(), "parse rejected"),
                            citations));
                } else if ("DOM2.UNMATCHED".equals(parsed.getReducerRuleId())) {
                    findings.add(finding("AMBIGUOUS_DOM2_SEMANTIC_REDUCTION", "LOW",
                            "Domestic NOTAM parsed as a record but did not match an implemented DOM2 semantic reducer; keep as constraint with retained raw text.",
                            citations));
                }
            } catch (RuntimeException ex) {
                findings.add(finding("MALFORMED_DOMESTIC_NOTAM", "MEDIUM",
                        "Domestic NOTAM semantic scan failed: " + ex.getMessage(),
                        AgentSupport.citations(impact)));
            }
        }
        return findings;
    }

    private List<AgentFinding> altrvGrammarFindings(List<ProductDtos.MessageSummary> messages,
                                                    ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        for (ProductDtos.MessageSummary message : messages) {
            String raw = message.getRawText() == null ? "" : message.getRawText();
            if (!looksLikeAltrv(message, raw)) {
                continue;
            }
            try {
                AltrvParseResult parsed = altrvParser.parse(raw);
                List<AgentSourceCitation> citations = java.util.Collections.singletonList(AgentSupport.citation(
                        value(message.getFamily(), "CARF_ALTRV"),
                        value(message.getId(), "carf-altrv"),
                        value(message.getSubject(), "CARF/ALTRV"),
                        message.getId() == null ? "/explorer" : "/messages/" + message.getId()));
                if (!parsed.isAccepted()) {
                    findings.add(finding("MALFORMED_ALTRV_GRAMMAR", "MEDIUM",
                            "CARF/ALTRV text did not satisfy modern ALTRV.g-derived parsing checks; retained raw text and diagnostics should be reviewed.",
                            citations));
                }
                if (parsed.getMessage() != null && parsed.getMessage().getAreas() != null
                        && parsed.getMessage().getAreas().stream().anyMatch(area -> area.getGeometryIntent() == null)) {
                    findings.add(finding("ALTRV_AREA_MISSING_GEOMETRY_INTENT", "LOW",
                            "One or more ALTRV areas parsed without explicit geometry intent metadata.",
                            citations));
                }
            } catch (RuntimeException ex) {
                findings.add(finding("MALFORMED_ALTRV_GRAMMAR", "MEDIUM",
                        "CARF/ALTRV grammar scan failed: " + ex.getMessage(),
                        AgentSupport.citations(impact)));
            }
        }
        return findings;
    }

    private List<AgentFinding> icaoNotamFieldFindings(List<ProductDtos.MessageSummary> messages,
                                                      ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        for (ProductDtos.MessageSummary message : messages) {
            String raw = message.getRawText() == null ? "" : message.getRawText();
            if (!looksLikeIcaoOrCanadianNotam(message, raw)) {
                continue;
            }
            try {
                NotamFieldParseResult fields = notamAirspaceParser.parseFields(raw);
                List<AgentSourceCitation> citations = java.util.Collections.singletonList(AgentSupport.citation(
                        value(message.getFamily(), "NOTAM"),
                        value(message.getId(), "icao-notam"),
                        value(message.getSubject(), "ICAO/Canadian NOTAM"),
                        message.getId() == null ? "/notams" : "/messages/" + message.getId()));
                if (!fields.isHasGeometry()) {
                    findings.add(finding("MISSING_NOTAM_GEOMETRY", "MEDIUM",
                            value(fields.getNotamType(), "NOTAM")
                                    + " retained Q/A/B/C/D/E/F/G metadata but has no route-usable geometry; keep as constraint and review raw text/diagnostics.",
                            citations));
                }
                if (fields.getQField() != null && fields.getAccountability() == null) {
                    findings.add(finding("AMBIGUOUS_ICAO_NOTAM_Q_FIELD", "LOW",
                            value(fields.getNotamType(), "NOTAM")
                                    + " has an empty FIR/accountability prefix in the Q field; source should be reviewed before route-impact use.",
                            citations));
                }
            } catch (RuntimeException ex) {
                findings.add(finding("MALFORMED_ICAO_NOTAM", "MEDIUM",
                        "ICAO/Canadian NOTAM field scan failed: " + ex.getMessage(),
                        AgentSupport.citations(impact)));
            }
        }
        return findings;
    }

    private List<AgentFinding> serviceCommandFindings(ProductDtos.RouteImpactSummary impact) {
        List<AgentFinding> findings = new ArrayList<>();
        for (ProductDtos.FeedArtifactSummary artifact : productService.feedArtifacts()) {
            for (ProductDtos.FeedTransactionSummary transaction : productService.feedTransactions(artifact.getId())) {
                if (transaction.getServiceCommandType() == null) {
                    continue;
                }
                List<AgentSourceCitation> citations = java.util.Collections.singletonList(AgentSupport.citation(
                        "USNS_" + transaction.getServiceCommandType(),
                        transaction.getId(),
                        value(transaction.getServiceCommandOperation(), transaction.getType()),
                        "/feed/" + artifact.getId()));
                if (!transaction.isServiceCommandAccepted() || !transaction.getErrors().isEmpty()) {
                    findings.add(finding("MALFORMED_USNS_SERVICE_COMMAND", "MEDIUM",
                            transaction.getServiceCommandType() + " command retained with parse diagnostics; no table mutation or external side effect was applied.",
                            citations));
                }
                if ("TABLE".equals(transaction.getServiceCommandType())) {
                    findings.add(finding("USNS_TABLE_COMMAND_RETAINED_ONLY", "INFO",
                            "USNS table command was classified and retained for audit, but local Airspace does not apply legacy table mutation semantics.",
                            citations));
                }
            }
        }
        return findings;
    }

    private boolean looksLikeDomesticNotam(ProductDtos.MessageSummary message, String raw) {
        String text = (value(message.getFamily(), "") + " " + value(message.getSubject(), "") + " " + raw)
                .toUpperCase(Locale.US);
        return raw.trim().startsWith("!") || text.contains("DOMESTIC") || text.contains("NOTAM");
    }

    private boolean looksLikeIcaoOrCanadianNotam(ProductDtos.MessageSummary message, String raw) {
        String text = (value(message.getFamily(), "") + " " + value(message.getSubject(), "") + " " + raw)
                .toUpperCase(Locale.US);
        return !raw.trim().startsWith("!") && containsAny(text, "NOTAMN", "NOTAMR", "NOTAMC", "NOTAMJ");
    }

    private boolean looksLikeAltrv(ProductDtos.MessageSummary message, String raw) {
        String text = (value(message.getFamily(), "") + " " + value(message.getSubject(), "") + " " + raw)
                .toUpperCase(Locale.US);
        return containsAny(text, "CARF", "ALTRV") || text.matches("(?s).*\\bA\\.\\s+.+\\bD\\.\\s+.+");
    }

    private String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
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
