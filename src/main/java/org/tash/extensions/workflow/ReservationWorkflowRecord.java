package org.tash.extensions.workflow;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.carf.api.CarfAnalysisResult;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ReservationWorkflowRecord {
    private String id;
    private ReservationDraft draft;
    private ReservationWorkflowState state;
    private ReservationWorkflowState stateBeforeLock;
    private String lockOwner;
    private ZonedDateTime lockedAt;
    private CarfAnalysisResult lastAnalysis;
    @Builder.Default
    private List<ConflictReviewItem> conflictReviews = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
    @Builder.Default
    private List<ReservationWorkflowAuditEvent> auditEvents = new ArrayList<>();
}
