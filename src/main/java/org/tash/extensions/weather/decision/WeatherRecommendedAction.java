package org.tash.extensions.weather.decision;

public enum WeatherRecommendedAction {
    NONE,
    MONITOR,
    AVOID_CELL,
    REROUTE_AROUND_WEATHER,
    CLIMB_OR_DESCEND,
    DELAY_DEPARTURE,
    HOLD_FOR_UPDATE,
    ROUTE_BLOCKED
}
