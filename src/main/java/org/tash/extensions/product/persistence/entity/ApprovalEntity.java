package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Table;

import java.util.UUID;

@Entity
@Table(name = "ops_approval")
public class ApprovalEntity extends BaseOperationalEntity {
    @Column(name = "reservation_id")
    private UUID reservationId;
    @Column(name = "approval_type", nullable = false)
    private String approvalType;
    @Column(name = "status", nullable = false)
    private String status;
    @Column(name = "actor")
    private String actor;
    @Column(name = "note", columnDefinition = "TEXT")
    private String note;

    public UUID getReservationId() { return reservationId; }
    public void setReservationId(UUID reservationId) { this.reservationId = reservationId; }
    public String getApprovalType() { return approvalType; }
    public void setApprovalType(String approvalType) { this.approvalType = approvalType; }
    public String getStatus() { return status; }
    public void setStatus(String status) { this.status = status; }
    public String getActor() { return actor; }
    public void setActor(String actor) { this.actor = actor; }
    public String getNote() { return note; }
    public void setNote(String note) { this.note = note; }
}
