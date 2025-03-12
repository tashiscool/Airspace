package org.tash.core.routing.lib;

/**
 * Factory for creating edges between vertices
 * Used by algorithms that dynamically create edges
 *
 * @param <V> Type of vertex
 * @param <E> Type of edge
 */
public interface EdgeFactory<V, E> {
    /**
     * Create an edge between two vertices
     *
     * @param from Starting vertex
     * @param to   Ending vertex
     * @return The created edge
     */
    E createEdge(V from, V to);
}
