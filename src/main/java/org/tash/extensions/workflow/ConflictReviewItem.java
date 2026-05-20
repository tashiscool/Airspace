package org.tash.extensions.workflow;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ConflictReviewItem {
    private String conflictId;
    private String firstReservationId;
    private String secondReservationId;
    private boolean reviewed;
    private boolean accepted;
    private String reviewer;
    private String note;
}
