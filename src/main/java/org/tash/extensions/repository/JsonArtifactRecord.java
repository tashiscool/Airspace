package org.tash.extensions.repository;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class JsonArtifactRecord {
    private String id;
    private String type;
    private String json;
    private ZonedDateTime storedAt;
}
