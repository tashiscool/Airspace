package org.tash.core.routing.lib;

/**
 * Heuristic function interface for informed search algorithms like A*
 * Estimates the cost from a vertex to the goal
 *
 * @param <V> Type of vertex
 */
public interface HeuristicFunction<V> {
    /**
     * Estimate the cost from a vertex to the goal
     *
     * @param current Current vertex
     * @param goal    Goal vertex
     * @return Estimated cost to goal
     */
    double estimate(V current, V goal);
}
