package org.tash.extensions.workflow;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ReservationDraft {
    private String id;
    private String rawText;
    private String createdBy;
    private String updatedBy;
    private ZonedDateTime createdAt;
    private ZonedDateTime updatedAt;
}
