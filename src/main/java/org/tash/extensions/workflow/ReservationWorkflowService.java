package org.tash.extensions.workflow;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.reservation.ReservationConflict;

import java.time.Clock;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

@ApplicationScoped
public class ReservationWorkflowService {
    private final ReservationWorkflowRepository repository;
    private final CarfAnalysisService analysisService;
    private final Clock clock;
    private final Duration staleLockThreshold;

    @Inject
    public ReservationWorkflowService(InMemoryReservationWorkflowRepository repository) {
        this(repository, new CarfAnalysisService(), Clock.systemUTC(), Duration.ofMinutes(20));
    }

    public ReservationWorkflowService(ReservationWorkflowRepository repository,
                                      CarfAnalysisService analysisService,
                                      Clock clock,
                                      Duration staleLockThreshold) {
        this.repository = repository;
        this.analysisService = analysisService;
        this.clock = clock;
        this.staleLockThreshold = staleLockThreshold;
    }

    public ReservationWorkflowResult createDraft(String rawText, String actor) {
        ZonedDateTime now = now();
        String id = UUID.randomUUID().toString();
        ReservationDraft draft = ReservationDraft.builder()
                .id(id)
                .rawText(rawText)
                .createdBy(actor)
                .updatedBy(actor)
                .createdAt(now)
                .updatedAt(now)
                .build();
        ReservationWorkflowRecord record = ReservationWorkflowRecord.builder()
                .id(id)
                .draft(draft)
                .state(ReservationWorkflowState.DRAFT)
                .build();
        record.getAuditEvents().add(event(ReservationWorkflowCommand.CREATE_DRAFT, actor, "draft-created"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult updateDraft(String id, String rawText, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() != ReservationWorkflowState.DRAFT && record.getState() != ReservationWorkflowState.VALIDATED) {
            return rejected(record, "Draft can only be updated while DRAFT or VALIDATED");
        }
        record.getDraft().setRawText(rawText);
        record.getDraft().setUpdatedBy(actor);
        record.getDraft().setUpdatedAt(now());
        record.setState(ReservationWorkflowState.DRAFT);
        record.setLastAnalysis(null);
        record.getConflictReviews().clear();
        record.getDiagnostics().clear();
        record.getAuditEvents().add(event(ReservationWorkflowCommand.UPDATE_DRAFT, actor, "draft-updated"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult validate(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getDraft() == null || isBlank(record.getDraft().getRawText())) {
            return rejected(record, "Reservation draft has no raw CARF/ALTRV text");
        }
        CarfAnalysisResult analysis = analysisService.parseValidateMap(record.getDraft().getRawText());
        CarfAnalysisResult withConflicts = analysisService.analyzeConflicts(analysis.getReservations());
        analysis.setConflicts(withConflicts.getConflicts());
        record.setLastAnalysis(analysis);
        record.setDiagnostics(new ArrayList<>(analysis.getDiagnostics()));
        record.setConflictReviews(reviewsFor(analysis.getConflicts()));
        record.setState(analysis.isAccepted() ? ReservationWorkflowState.VALIDATED : ReservationWorkflowState.DRAFT);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.VALIDATE, actor,
                analysis.isAccepted() ? "validation-accepted" : "validation-rejected"));
        repository.save(record);
        return ReservationWorkflowResult.builder()
                .accepted(analysis.isAccepted())
                .record(record)
                .diagnostics(new ArrayList<>(record.getDiagnostics()))
                .build();
    }

    public ReservationWorkflowResult submit(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() != ReservationWorkflowState.VALIDATED) {
            return rejected(record, "Submit requires a VALIDATED reservation");
        }
        record.setState(ReservationWorkflowState.SUBMITTED);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.SUBMIT, actor, "submitted"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult approve(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() != ReservationWorkflowState.SUBMITTED) {
            return rejected(record, "Approve requires a SUBMITTED reservation");
        }
        if (record.getLastAnalysis() == null || !record.getLastAnalysis().isAccepted()) {
            return rejected(record, "Approve requires accepted validation");
        }
        if (hasUnacceptedConflict(record)) {
            return rejected(record, "Approve requires every blocking conflict to be reviewed and accepted");
        }
        record.setState(ReservationWorkflowState.APPROVED);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.APPROVE, actor, "approved"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult reject(String id, String actor, String note) {
        ReservationWorkflowRecord record = require(id);
        if (terminal(record)) {
            return rejected(record, "Terminal reservation cannot be rejected");
        }
        record.setState(ReservationWorkflowState.REJECTED);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.REJECT, actor, note));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult cancel(String id, String actor, String note) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() == ReservationWorkflowState.COMPLETED) {
            return rejected(record, "Completed reservation cannot be cancelled");
        }
        record.setState(ReservationWorkflowState.CANCELLED);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.CANCEL, actor, note));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult complete(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() != ReservationWorkflowState.APPROVED) {
            return rejected(record, "Complete requires an APPROVED reservation");
        }
        record.setState(ReservationWorkflowState.COMPLETED);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.COMPLETE, actor, "completed"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult lock(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() == ReservationWorkflowState.LOCKED) {
            if (isStale(record)) {
                record.setState(ReservationWorkflowState.STALE_LOCK);
                repository.save(record);
            } else {
                return rejected(record, "Reservation is already locked by " + record.getLockOwner());
            }
        }
        if (terminal(record)) {
            return rejected(record, "Terminal reservation cannot be locked");
        }
        record.setStateBeforeLock(record.getState());
        record.setState(ReservationWorkflowState.LOCKED);
        record.setLockOwner(actor);
        record.setLockedAt(now());
        record.getAuditEvents().add(event(ReservationWorkflowCommand.LOCK, actor, "locked"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult unlock(String id, String actor) {
        ReservationWorkflowRecord record = require(id);
        if (record.getState() != ReservationWorkflowState.LOCKED
                && record.getState() != ReservationWorkflowState.STALE_LOCK) {
            return rejected(record, "Reservation is not locked");
        }
        ReservationWorkflowState restored = record.getStateBeforeLock() == null
                ? ReservationWorkflowState.DRAFT
                : record.getStateBeforeLock();
        record.setState(restored);
        record.setStateBeforeLock(null);
        record.setLockOwner(null);
        record.setLockedAt(null);
        record.getAuditEvents().add(event(ReservationWorkflowCommand.UNLOCK, actor, "unlocked"));
        repository.save(record);
        return accepted(record);
    }

    public ReservationWorkflowResult reviewConflict(String id, String conflictId, boolean accepted,
                                                    String reviewer, String note) {
        ReservationWorkflowRecord record = require(id);
        for (ConflictReviewItem review : record.getConflictReviews()) {
            if (review.getConflictId().equals(conflictId)) {
                review.setReviewed(true);
                review.setAccepted(accepted);
                review.setReviewer(reviewer);
                review.setNote(note);
                repository.save(record);
                return accepted(record);
            }
        }
        return rejected(record, "Unknown conflict review item: " + conflictId);
    }

    public ReservationWorkflowRecord findById(String id) {
        return require(id);
    }

    private ReservationWorkflowRecord require(String id) {
        return repository.findById(id)
                .orElseThrow(() -> new IllegalArgumentException("Unknown reservation workflow record: " + id));
    }

    private List<ConflictReviewItem> reviewsFor(List<ReservationConflict> conflicts) {
        List<ConflictReviewItem> reviews = new ArrayList<>();
        if (conflicts == null) {
            return reviews;
        }
        for (ReservationConflict conflict : conflicts) {
            String conflictId = conflict.getFirst().getId() + "__" + conflict.getSecond().getId();
            reviews.add(ConflictReviewItem.builder()
                    .conflictId(conflictId)
                    .firstReservationId(conflict.getFirst().getId())
                    .secondReservationId(conflict.getSecond().getId())
                    .reviewed(false)
                    .accepted(false)
                    .build());
        }
        return reviews;
    }

    private boolean hasUnacceptedConflict(ReservationWorkflowRecord record) {
        for (ConflictReviewItem review : record.getConflictReviews()) {
            if (!review.isReviewed() || !review.isAccepted()) {
                return true;
            }
        }
        return false;
    }

    private boolean isStale(ReservationWorkflowRecord record) {
        return record.getLockedAt() != null
                && record.getLockedAt().plus(staleLockThreshold).isBefore(now());
    }

    private boolean terminal(ReservationWorkflowRecord record) {
        return record.getState() == ReservationWorkflowState.CANCELLED
                || record.getState() == ReservationWorkflowState.COMPLETED
                || record.getState() == ReservationWorkflowState.REJECTED;
    }

    private ReservationWorkflowResult accepted(ReservationWorkflowRecord record) {
        return ReservationWorkflowResult.builder()
                .accepted(true)
                .record(record)
                .diagnostics(new ArrayList<>(record.getDiagnostics()))
                .build();
    }

    private ReservationWorkflowResult rejected(ReservationWorkflowRecord record, String diagnostic) {
        List<String> diagnostics = new ArrayList<>(record.getDiagnostics());
        diagnostics.add(diagnostic);
        record.setDiagnostics(diagnostics);
        return ReservationWorkflowResult.builder()
                .accepted(false)
                .record(record)
                .diagnostics(diagnostics)
                .build();
    }

    private ReservationWorkflowAuditEvent event(ReservationWorkflowCommand command, String actor, String note) {
        return ReservationWorkflowAuditEvent.builder()
                .command(command)
                .actor(actor)
                .timestamp(now())
                .note(note)
                .build();
    }

    private ZonedDateTime now() {
        return ZonedDateTime.now(clock);
    }

    private boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
