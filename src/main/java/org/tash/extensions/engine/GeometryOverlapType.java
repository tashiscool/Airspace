package org.tash.extensions.engine;

public enum GeometryOverlapType {
    NONE,
    POINT_TOUCH,
    SEGMENT_CROSSING,
    CONTAINMENT,
    BBOX_PREFILTER_ONLY,
    UNKNOWN_GEOMETRY
}
