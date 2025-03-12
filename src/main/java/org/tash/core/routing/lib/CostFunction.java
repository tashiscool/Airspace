package org.tash.core.routing.lib;

/**
 * Cost function interface for path planning algorithms
 * Calculates the cost of traversing an edge
 *
 * @param <V> Type of vertex
 * @param <E> Type of edge
 */
public interface CostFunction<V, E> {
    /**
     * Calculate the cost of traversing an edge
     *
     * @param edge Edge to calculate cost for
     * @return Cost value (higher means more costly)
     */
    double getCost(E edge);

    /**
     * Calculate the cost between two vertices (typically used for heuristics)
     *
     * @param from Starting vertex
     * @param to   Ending vertex
     * @return Estimated cost between vertices
     */
    double getCost(V from, V to);
}
