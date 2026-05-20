package org.tash.extensions.notam.access;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class NotamAccessRequest {
    private String privilege;
    private String accountId;
    private String series;
    private String source;
    private List<String> locationIds;
}
