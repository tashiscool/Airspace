package org.tash.extensions.workflow;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

@ApplicationScoped
public class InMemoryReservationWorkflowRepository implements ReservationWorkflowRepository {
    private final Map<String, ReservationWorkflowRecord> records = Collections.synchronizedMap(new LinkedHashMap<>());

    @Override
    public ReservationWorkflowRecord save(ReservationWorkflowRecord record) {
        records.put(record.getId(), record);
        return record;
    }

    @Override
    public Optional<ReservationWorkflowRecord> findById(String id) {
        return Optional.ofNullable(records.get(id));
    }

    @Override
    public List<ReservationWorkflowRecord> findOpen() {
        List<ReservationWorkflowRecord> open = new ArrayList<>();
        synchronized (records) {
            for (ReservationWorkflowRecord record : records.values()) {
                if (record.getState() != ReservationWorkflowState.CANCELLED
                        && record.getState() != ReservationWorkflowState.COMPLETED
                        && record.getState() != ReservationWorkflowState.REJECTED) {
                    open.add(record);
                }
            }
        }
        return open;
    }

    @Override
    public ReservationWorkflowRecord appendAuditEvent(String id, ReservationWorkflowAuditEvent event) {
        ReservationWorkflowRecord record = records.get(id);
        if (record == null) {
            throw new IllegalArgumentException("Unknown reservation workflow record: " + id);
        }
        record.getAuditEvents().add(event);
        return save(record);
    }
}
