package org.tash.extensions.agentic;

import org.tash.extensions.product.dto.ProductDtos;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

final class AgentSupport {
    private AgentSupport() {
    }

    static String id(String prefix, String seed) {
        return prefix + "-" + UUID.nameUUIDFromBytes(String.valueOf(seed).getBytes(java.nio.charset.StandardCharsets.UTF_8));
    }

    static AgentSourceCitation citation(String family, String id, String label, String route) {
        return AgentSourceCitation.builder()
                .sourceFamily(family)
                .sourceId(id)
                .label(label)
                .route(route)
                .build();
    }

    static AgentSourceCitation citation(String ref) {
        String safe = ref == null ? "unknown" : ref;
        String family = safe.contains(":") ? safe.substring(0, safe.indexOf(':')) : "SOURCE";
        String id = safe.contains(":") ? safe.substring(safe.indexOf(':') + 1) : safe;
        return citation(family.toUpperCase(Locale.US), id, safe, routeFor(family, id));
    }

    static List<AgentSourceCitation> citations(List<String> refs) {
        if (refs == null || refs.isEmpty()) {
            return Collections.singletonList(citation("ENGINE", "derived", "Derived from engine output", "/decisions/latest"));
        }
        List<AgentSourceCitation> citations = new ArrayList<>();
        for (String ref : refs) {
            citations.add(citation(ref));
        }
        return citations;
    }

    static List<AgentSourceCitation> citations(ProductDtos.RouteImpactSummary impact) {
        return citations(impact == null ? null : impact.getSourceRefs());
    }

    static String routeFor(String family, String id) {
        String normalized = family == null ? "" : family.toUpperCase(Locale.US);
        if (normalized.contains("MISSION")) {
            return "/missions/" + id;
        }
        if (normalized.contains("RESERVATION")) {
            return "/missions/" + id + "/reservations/" + id;
        }
        if (normalized.contains("MESSAGE") || normalized.contains("USNS") || normalized.contains("NOTAM")
                || normalized.contains("WEATHER") || normalized.contains("PIREP") || normalized.contains("SIGMET")
                || normalized.contains("AIRMET") || normalized.contains("METAR") || normalized.contains("TAF")) {
            return "/messages/" + id;
        }
        if (normalized.contains("FEED")) {
            return "/feed/" + id;
        }
        if (normalized.contains("DECISION")) {
            return "/decisions/" + id;
        }
        return null;
    }

    static String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
    }
}
