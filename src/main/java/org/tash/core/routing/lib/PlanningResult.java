package org.tash.core.routing.lib;

import java.util.List;
import java.util.Map;

/**
 * Class representing planning results with additional metadata
 *
 * @param <V> Type of vertex
 * @param <E> Type of edge
 */
public class PlanningResult<V, E> {
    /**
     * The path as a list of edges
     */
    private final List<E> path;

    /**
     * Total cost of the path
     */
    private final double totalCost;

    /**
     * Path metadata such as waypoints, estimated time, etc.
     */
    private final Map<String, Object> metadata;

    /**
     * Number of nodes explored during planning
     */
    private final int nodesExplored;

    /**
     * Time taken to compute the path in milliseconds
     */
    private final long computationTimeMs;

    public PlanningResult(List<E> path, double totalCost,
                          Map<String, Object> metadata,
                          int nodesExplored,
                          long computationTimeMs) {
        this.path = path;
        this.totalCost = totalCost;
        this.metadata = metadata;
        this.nodesExplored = nodesExplored;
        this.computationTimeMs = computationTimeMs;
    }

    public List<E> getPath() {
        return path;
    }

    public double getTotalCost() {
        return totalCost;
    }

    public Map<String, Object> getMetadata() {
        return metadata;
    }

    public int getNodesExplored() {
        return nodesExplored;
    }

    public long getComputationTimeMs() {
        return computationTimeMs;
    }

    public boolean isPathFound() {
        return path != null && !path.isEmpty();
    }
}
