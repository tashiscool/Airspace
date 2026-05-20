package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;

import java.util.List;
import java.util.Map;

@Data
@Builder
public class AltrvRouteGraph {
    private Graph<String, DefaultEdge> graph;
    private List<String> nodeOrder;
    private Map<String, AltrvRouteGraphVertex> vertices;
    private List<AltrvRouteGraphEdge> edges;
}
