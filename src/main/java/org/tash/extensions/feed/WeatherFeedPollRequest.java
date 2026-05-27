package org.tash.extensions.feed;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;

@Data
public class WeatherFeedPollRequest {
    private List<String> products = new ArrayList<>();
    private Integer hoursBeforeNow;
    private Integer maxResults;
}
