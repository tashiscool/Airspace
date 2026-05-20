package org.tash.extensions.product.application;

import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.workflow.ReservationWorkflowRecord;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

class ProductRecordStore {
    final Map<String, ProductDtos.MissionSummary> missions = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ReservationWorkflowRecord> reservations = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, String> reservationMissionIds = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ProductDtos.MessageSummary> messages = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, OperationalFeedIngestResult> feedArtifacts = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ProductDtos.DecisionSummary> decisions = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ProductDtos.HistoryEventSummary> history = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ProductDtos.ReferencePointSummary> referencePoints = Collections.synchronizedMap(new LinkedHashMap<>());
    final Map<String, ProductDtos.ReservationSupplementSummary> supplements = Collections.synchronizedMap(new LinkedHashMap<>());

    ProductDtos.HistoryEventSummary history(String aggregateType, String aggregateId,
                                            String eventType, String actor, String note) {
        ProductDtos.HistoryEventSummary event = ProductDtos.HistoryEventSummary.builder()
                .id(UUID.randomUUID().toString())
                .aggregateType(aggregateType)
                .aggregateId(aggregateId)
                .eventType(eventType)
                .actor(actor == null ? "system" : actor)
                .note(note)
                .createdAt(ZonedDateTime.now(ZoneOffset.UTC))
                .build();
        history.put(event.getId(), event);
        return event;
    }

    List<ProductDtos.HistoryEventSummary> historyFor(String aggregateType, String aggregateId) {
        List<ProductDtos.HistoryEventSummary> out = new ArrayList<>();
        synchronized (history) {
            for (ProductDtos.HistoryEventSummary event : history.values()) {
                if ((aggregateType == null || aggregateType.equals(event.getAggregateType()))
                        && (aggregateId == null || aggregateId.equals(event.getAggregateId()))) {
                    out.add(event);
                }
            }
        }
        return out;
    }
}
