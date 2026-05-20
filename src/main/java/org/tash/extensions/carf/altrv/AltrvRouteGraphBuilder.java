package org.tash.extensions.carf.altrv;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleGraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class AltrvRouteGraphBuilder {
    public AltrvRouteGraph build(AltrvMessage message) {
        Graph<String, DefaultEdge> graph = new SimpleGraph<>(DefaultEdge.class);
        List<String> order = new ArrayList<>();
        Map<String, AltrvRouteGraphVertex> vertices = new LinkedHashMap<>();
        List<AltrvRouteGraphEdge> edges = new ArrayList<>();
        if (message == null || message.getRouteGroups() == null) {
            return AltrvRouteGraph.builder()
                    .graph(graph)
                    .nodeOrder(Collections.emptyList())
                    .vertices(Collections.emptyMap())
                    .edges(Collections.emptyList())
                    .build();
        }
        String departure = "DEP:" + value(message.getLocation(), "UNKNOWN");
        addVertex(graph, order, vertices, AltrvRouteGraphVertex.builder()
                .id(departure)
                .type(AltrvRouteGraphVertexType.DEPARTURE)
                .label(value(message.getLocation(), "departure"))
                .build());
        for (AltrvRouteGroup group : message.getRouteGroups()) {
            for (AltrvRoute route : group.getRoutes()) {
                String routeStart = "ROUTE:" + route.getId() + ":" + route.getKind();
                addVertex(graph, order, vertices, AltrvRouteGraphVertex.builder()
                        .id(routeStart)
                        .type(AltrvRouteGraphVertexType.ROUTE_START)
                        .routeId(route.getId())
                        .label(route.getKind() == null ? "route" : route.getKind().name())
                        .build());
                addEdge(graph, edges, departure, routeStart, message.getCallsigns(), "departure-to-route");
                String previous = routeStart;
                for (AltrvRoutePoint point : route.getPoints()) {
                    String node = group.getId() + ":" + route.getId() + ":" + point.getId() + ":" + point.getTimeOffset();
                    addVertex(graph, order, vertices, AltrvRouteGraphVertex.builder()
                            .id(node)
                            .type(AltrvRouteGraphVertexType.ROUTE_GROUP)
                            .routeId(route.getId())
                            .label(point.getId())
                            .sourceSpan(point.getSourceSpan())
                            .build());
                    addEdge(graph, edges, previous, node, message.getCallsigns(), "route-sequence");
                    previous = node;
                }
                for (AltrvRouteEvent event : route.getEvents()) {
                    String eventNode = group.getId() + ":" + route.getId() + ":" + event.getEventId();
                    addVertex(graph, order, vertices, AltrvRouteGraphVertex.builder()
                            .id(eventNode)
                            .type(eventVertexType(event.getType()))
                            .routeId(route.getId())
                            .eventId(event.getEventId())
                            .label(event.getType().name())
                            .sourceSpan(event.getSourceSpan())
                            .build());
                    addEdge(graph, edges, previous, eventNode, message.getCallsigns(),
                            "route-event:" + event.getType().name());
                    previous = eventNode;
                }
                int destinationIndex = 0;
                for (AltrvDestination destination : safe(message.getDestinations())) {
                    String destNode = "DEST:" + destinationIndex++ + ":" + value(destination.getFixId(), "UNKNOWN");
                    addVertex(graph, order, vertices, AltrvRouteGraphVertex.builder()
                            .id(destNode)
                            .type(AltrvRouteGraphVertexType.DESTINATION)
                            .routeId(route.getId())
                            .label(destination.getFixId())
                            .sourceSpan(destination.getSourceSpan())
                            .build());
                    addEdge(graph, edges, previous, destNode, destination.getCallsigns(), "route-to-destination");
                }
            }
        }
        return AltrvRouteGraph.builder()
                .graph(graph)
                .nodeOrder(Collections.unmodifiableList(order))
                .vertices(Collections.unmodifiableMap(vertices))
                .edges(Collections.unmodifiableList(edges))
                .build();
    }

    private void addVertex(Graph<String, DefaultEdge> graph,
                           List<String> order,
                           Map<String, AltrvRouteGraphVertex> vertices,
                           AltrvRouteGraphVertex vertex) {
        if (!graph.containsVertex(vertex.getId())) {
            graph.addVertex(vertex.getId());
            order.add(vertex.getId());
        }
        vertices.put(vertex.getId(), vertex);
    }

    private void addEdge(Graph<String, DefaultEdge> graph,
                         List<AltrvRouteGraphEdge> edges,
                         String source,
                         String target,
                         List<AltrvCallsign> callsigns,
                         String reason) {
        if (source == null || target == null || source.equals(target)) {
            return;
        }
        if (graph.containsVertex(source) && graph.containsVertex(target)) {
            graph.addEdge(source, target);
            edges.add(AltrvRouteGraphEdge.builder()
                    .sourceId(source)
                    .targetId(target)
                    .callsigns(callsigns == null ? Collections.emptyList() : callsigns)
                    .reason(reason)
                    .build());
        }
    }

    private String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
    }

    private AltrvRouteGraphVertexType eventVertexType(AltrvRouteEventType type) {
        if (type == AltrvRouteEventType.JOIN || type == AltrvRouteEventType.JOIN_COMMON_ROUTE
                || type == AltrvRouteEventType.RECEIVER_COMMON_ROUTE) {
            return AltrvRouteGraphVertexType.JOIN;
        }
        if (type == AltrvRouteEventType.ROUTE_BRANCH || type == AltrvRouteEventType.BRANCH_CLOSE
                || type == AltrvRouteEventType.BRANCH_MERGE || type == AltrvRouteEventType.LEAVE) {
            return AltrvRouteGraphVertexType.BRANCH;
        }
        if (type == AltrvRouteEventType.ROUTE_COMMON) {
            return AltrvRouteGraphVertexType.COMMON_ROUTE;
        }
        return AltrvRouteGraphVertexType.EVENT;
    }

    private <T> List<T> safe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
