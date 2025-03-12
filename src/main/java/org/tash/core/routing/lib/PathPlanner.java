package org.tash.core.routing.lib;

import org.tash.core.SpatialElement;

import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Base interface for all path planning algorithms
 * 
 * @param <V> Type of vertex (typically SpatialPoint)
 * @param <E> Type of edge (typically TrajectorySegment)
 */
public interface PathPlanner<V, E> {
    /**
     * Find a path from start to goal
     * 
     * @param start Starting vertex
     * @param goal Goal vertex
     * @return List of edges forming the path, or empty list if no path found
     */
    List<E> findPath(V start, V goal);
    
    /**
     * Find a path from start to goal avoiding obstacles
     * 
     * @param start Starting vertex
     * @param goal Goal vertex
     * @param obstacles Set of elements to avoid
     * @return List of edges forming the path, or empty list if no path found
     */
    List<E> findPath(V start, V goal, Set<? extends SpatialElement> obstacles);
    
    /**
     * Find a path from start to goal with constraints
     * 
     * @param start Starting vertex
     * @param goal Goal vertex
     * @param constraints Predicate that returns true if an edge is valid (satisfies constraints)
     * @return List of edges forming the path, or empty list if no path found
     */
    List<E> findPathWithConstraints(V start, V goal, Predicate<E> constraints);
}

