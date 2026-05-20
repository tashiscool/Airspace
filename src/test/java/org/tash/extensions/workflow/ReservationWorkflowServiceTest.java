package org.tash.extensions.workflow;

import org.junit.jupiter.api.Test;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.carf.api.CarfAnalysisResult;

import java.nio.file.Path;
import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;

import static org.junit.jupiter.api.Assertions.*;

class ReservationWorkflowServiceTest {
    @Test
    void draftValidateSubmitApproveAndCompleteWorkflow() {
        ReservationWorkflowService service = service();

        ReservationWorkflowResult created = service.createDraft(validMessage(), "planner");
        ReservationWorkflowResult validated = service.validate(created.getRecord().getId(), "planner");
        assertTrue(validated.isAccepted(), validated.getDiagnostics().toString());
        assertEquals(ReservationWorkflowState.VALIDATED, validated.getRecord().getState());
        ReservationWorkflowResult submitted = service.submit(created.getRecord().getId(), "planner");
        assertEquals(ReservationWorkflowState.SUBMITTED, submitted.getRecord().getState());
        ReservationWorkflowResult approved = service.approve(created.getRecord().getId(), "supervisor");
        assertEquals(ReservationWorkflowState.APPROVED, approved.getRecord().getState());
        ReservationWorkflowResult completed = service.complete(created.getRecord().getId(), "system");

        assertTrue(created.isAccepted());
        assertEquals(ReservationWorkflowState.COMPLETED, completed.getRecord().getState());
        assertFalse(completed.getRecord().getAuditEvents().isEmpty());
    }

    @Test
    void invalidDraftBlocksSubmit() {
        ReservationWorkflowService service = service();

        ReservationWorkflowResult created = service.createDraft("A. BAD\nD.\n", "planner");
        ReservationWorkflowResult validation = service.validate(created.getRecord().getId(), "planner");
        ReservationWorkflowResult submit = service.submit(created.getRecord().getId(), "planner");

        assertFalse(validation.isAccepted());
        assertFalse(submit.isAccepted());
        assertTrue(submit.getDiagnostics().stream().anyMatch(d -> d.contains("Submit requires")));
    }

    @Test
    void lockAndUnlockAreDeterministic() {
        InMemoryReservationWorkflowRepository repository = new InMemoryReservationWorkflowRepository();
        ReservationWorkflowService service = new ReservationWorkflowService(repository, new CarfAnalysisService(),
                Clock.fixed(Instant.parse("2010-01-01T00:00:00Z"), ZoneOffset.UTC), Duration.ofMinutes(20));
        String id = service.createDraft(validMessage(), "planner").getRecord().getId();

        ReservationWorkflowResult locked = service.lock(id, "planner");
        assertEquals(ReservationWorkflowState.LOCKED, locked.getRecord().getState());
        ReservationWorkflowResult duplicateLock = service.lock(id, "other");
        ReservationWorkflowResult unlocked = service.unlock(id, "planner");

        assertFalse(duplicateLock.isAccepted());
        assertEquals(ReservationWorkflowState.DRAFT, unlocked.getRecord().getState());
    }

    @Test
    void approvalRequiresBlockingConflictsToBeReviewedAndAccepted() {
        InMemoryReservationWorkflowRepository repository = new InMemoryReservationWorkflowRepository();
        ReservationWorkflowService service = new ReservationWorkflowService(repository, new CarfAnalysisService(),
                Clock.fixed(Instant.parse("2010-01-01T00:00:00Z"), ZoneOffset.UTC), Duration.ofMinutes(20));
        ReservationWorkflowRecord record = ReservationWorkflowRecord.builder()
                .id("WF-CONFLICT")
                .state(ReservationWorkflowState.SUBMITTED)
                .lastAnalysis(CarfAnalysisResult.builder().accepted(true).build())
                .build();
        record.getConflictReviews().add(ConflictReviewItem.builder()
                .conflictId("A__B")
                .firstReservationId("A")
                .secondReservationId("B")
                .build());
        repository.save(record);

        ReservationWorkflowResult blocked = service.approve("WF-CONFLICT", "supervisor");
        ReservationWorkflowResult reviewed = service.reviewConflict("WF-CONFLICT", "A__B", true,
                "supervisor", "operationally accepted");
        ReservationWorkflowResult approved = service.approve("WF-CONFLICT", "supervisor");

        assertFalse(blocked.isAccepted());
        assertTrue(reviewed.isAccepted());
        assertTrue(approved.isAccepted());
        assertEquals(ReservationWorkflowState.APPROVED, approved.getRecord().getState());
    }

    @Test
    void jsonRepositorySavesAndReloadsRecords() throws Exception {
        Path file = java.nio.file.Files.createTempFile("reservation-workflow", ".json");
        JsonFileReservationWorkflowRepository repository = new JsonFileReservationWorkflowRepository(file);
        ReservationWorkflowRecord record = ReservationWorkflowRecord.builder()
                .id("WF1")
                .state(ReservationWorkflowState.DRAFT)
                .draft(ReservationDraft.builder().id("WF1").rawText(validMessage()).build())
                .build();

        repository.save(record);
        JsonFileReservationWorkflowRepository reloaded = new JsonFileReservationWorkflowRepository(file);

        assertTrue(reloaded.findById("WF1").isPresent());
        assertEquals(ReservationWorkflowState.DRAFT, reloaded.findById("WF1").get().getState());
    }

    private ReservationWorkflowService service() {
        return new ReservationWorkflowService(new InMemoryReservationWorkflowRepository(), new CarfAnalysisService(),
                Clock.fixed(Instant.parse("2010-01-01T00:00:00Z"), ZoneOffset.UTC), Duration.ofMinutes(20));
    }

    private String validMessage() {
        return "A. TEST01 KZNY\n"
                + "B. 1F22/I\n"
                + "C. KZNY\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100\n"
                + "E. TEST01\n"
                + "F. ETD TEST01 021200 MAR 2010 AVANA 021300\n"
                + "G. TAS: 300 KTAS\n";
    }
}
