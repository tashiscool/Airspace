package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

import java.util.UUID;

@Entity
@Table(name = "ops_apreq")
public class ApreqEntity extends BaseOperationalEntity {
    @Column(name = "reservation_id")
    private UUID reservationId;
    @Column(name = "status", nullable = false)
    private String status = "DRAFT";
    @Column(name = "request_text", columnDefinition = "TEXT")
    private String requestText;
    @Column(name = "response_text", columnDefinition = "TEXT")
    private String responseText;

    public UUID getReservationId() { return reservationId; }
    public void setReservationId(UUID reservationId) { this.reservationId = reservationId; }
    public String getStatus() { return status; }
    public void setStatus(String status) { this.status = status; }
    public String getRequestText() { return requestText; }
    public void setRequestText(String requestText) { this.requestText = requestText; }
    public String getResponseText() { return responseText; }
    public void setResponseText(String responseText) { this.responseText = responseText; }
}
