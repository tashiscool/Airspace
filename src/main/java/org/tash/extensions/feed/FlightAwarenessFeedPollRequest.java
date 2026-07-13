package org.tash.extensions.feed;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
public class FlightAwarenessFeedPollRequest {
    private String providerId;
    private String transportMode;
    private String mode;
    private String rawResponseJson;
    private String bounds;
    private List<String> flights = new ArrayList<>();
    private List<String> callsigns = new ArrayList<>();
    private List<String> registrations = new ArrayList<>();
    private List<String> airports = new ArrayList<>();
    private List<String> routes = new ArrayList<>();
    private List<String> aircraft = new ArrayList<>();
    private List<String> altitudeRanges = new ArrayList<>();
    private List<String> categories = new ArrayList<>();
    private List<String> trackingSourceTypes = new ArrayList<>();
    private Long timestamp;
    private Integer maxResults;
    private boolean full = true;
    private String sourceId;
}
