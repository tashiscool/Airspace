package org.tash.core.routing.lib;

import org.tash.core.SpatialElement;

import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Interface for algorithms that can generate multiple alternative paths
 *
 * @param <V> Type of vertex
 * @param <E> Type of edge
 */
public interface MultiPathPlanner<V, E> extends PathPlanner<V, E> {
    /**
     * Find multiple paths from start to goal
     *
     * @param start    Starting vertex
     * @param goal     Goal vertex
     * @param numPaths Maximum number of paths to find
     * @return List of path alternatives, each being a list of edges
     */
    List<List<E>> findMultiplePaths(V start, V goal, int numPaths);

    /**
     * Find multiple paths from start to goal avoiding obstacles
     *
     * @param start     Starting vertex
     * @param goal      Goal vertex
     * @param obstacles Set of elements to avoid
     * @param numPaths  Maximum number of paths to find
     * @return List of path alternatives, each being a list of edges
     */
    List<List<E>> findMultiplePaths(V start, V goal, Set<? extends SpatialElement> obstacles, int numPaths);

    /**
     * Find multiple paths from start to goal with constraints
     *
     * @param start       Starting vertex
     * @param goal        Goal vertex
     * @param constraints Predicate that returns true if an edge is valid
     * @param numPaths    Maximum number of paths to find
     * @return List of path alternatives, each being a list of edges
     */
    List<List<E>> findMultiplePathsWithConstraints(V start, V goal, Predicate<E> constraints, int numPaths);
}
