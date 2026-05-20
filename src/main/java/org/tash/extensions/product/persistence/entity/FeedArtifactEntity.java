package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.UUID;

@Entity
@Table(name = "ops_feed_artifact")
public class FeedArtifactEntity {
    @jakarta.persistence.Id
    @Column(name = "id", nullable = false)
    private UUID id;
    @Column(name = "source_id", nullable = false)
    private String sourceId;
    @Column(name = "feed_type", nullable = false)
    private String feedType;
    @Column(name = "payload_hash", nullable = false)
    private String payloadHash;
    @Column(name = "raw_payload", columnDefinition = "TEXT")
    private String rawPayload;
    @Column(name = "diagnostics_json", columnDefinition = "TEXT")
    private String diagnosticsJson;
    @Column(name = "accepted", nullable = false)
    private boolean accepted;
    @Column(name = "received_at", nullable = false)
    private ZonedDateTime receivedAt;

    public UUID getId() { return id; }
    public void setId(UUID id) { this.id = id; }
    public String getSourceId() { return sourceId; }
    public void setSourceId(String sourceId) { this.sourceId = sourceId; }
    public String getFeedType() { return feedType; }
    public void setFeedType(String feedType) { this.feedType = feedType; }
    public String getPayloadHash() { return payloadHash; }
    public void setPayloadHash(String payloadHash) { this.payloadHash = payloadHash; }
    public String getRawPayload() { return rawPayload; }
    public void setRawPayload(String rawPayload) { this.rawPayload = rawPayload; }
    public String getDiagnosticsJson() { return diagnosticsJson; }
    public void setDiagnosticsJson(String diagnosticsJson) { this.diagnosticsJson = diagnosticsJson; }
    public boolean isAccepted() { return accepted; }
    public void setAccepted(boolean accepted) { this.accepted = accepted; }
    public ZonedDateTime getReceivedAt() { return receivedAt; }
    public void setReceivedAt(ZonedDateTime receivedAt) { this.receivedAt = receivedAt; }
}
