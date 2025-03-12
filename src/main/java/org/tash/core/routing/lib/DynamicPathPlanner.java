package org.tash.core.routing.lib;

import org.tash.core.SpatialElement;

import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Interface for dynamic path planning algorithms that can replan based on changing conditions
 *
 * @param <V> Type of vertex
 * @param <E> Type of edge
 */
public interface DynamicPathPlanner<V, E> extends PathPlanner<V, E> {
    /**
     * Replan the path based on the current position and new obstacles/constraints
     *
     * @param currentPosition Current position along the path
     * @param goal            Goal vertex
     * @param currentPath     Current planned path
     * @param obstacles       Updated set of obstacles
     * @return Replanned path from current position to goal
     */
    List<E> replanPath(V currentPosition, V goal, List<E> currentPath, Set<? extends SpatialElement> obstacles);

    /**
     * Replan the path with changed constraints
     *
     * @param currentPosition Current position along the path
     * @param goal            Goal vertex
     * @param currentPath     Current planned path
     * @param constraints     Updated constraints
     * @return Replanned path from current position to goal
     */
    List<E> replanPathWithConstraints(V currentPosition, V goal, List<E> currentPath, Predicate<E> constraints);
}
