package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.UUID;

@Entity
@Table(name = "ops_history_event")
public class HistoryEventEntity {
    @jakarta.persistence.Id
    @Column(name = "id", nullable = false)
    private UUID id;
    @Column(name = "aggregate_type", nullable = false)
    private String aggregateType;
    @Column(name = "aggregate_id", nullable = false)
    private String aggregateId;
    @Column(name = "event_type", nullable = false)
    private String eventType;
    @Column(name = "actor")
    private String actor;
    @Column(name = "note", columnDefinition = "TEXT")
    private String note;
    @Column(name = "event_json", columnDefinition = "TEXT")
    private String eventJson;
    @Column(name = "created_at", nullable = false)
    private ZonedDateTime createdAt;

    public UUID getId() { return id; }
    public void setId(UUID id) { this.id = id; }
    public String getAggregateType() { return aggregateType; }
    public void setAggregateType(String aggregateType) { this.aggregateType = aggregateType; }
    public String getAggregateId() { return aggregateId; }
    public void setAggregateId(String aggregateId) { this.aggregateId = aggregateId; }
    public String getEventType() { return eventType; }
    public void setEventType(String eventType) { this.eventType = eventType; }
    public String getActor() { return actor; }
    public void setActor(String actor) { this.actor = actor; }
    public String getNote() { return note; }
    public void setNote(String note) { this.note = note; }
    public String getEventJson() { return eventJson; }
    public void setEventJson(String eventJson) { this.eventJson = eventJson; }
    public ZonedDateTime getCreatedAt() { return createdAt; }
    public void setCreatedAt(ZonedDateTime createdAt) { this.createdAt = createdAt; }
}
