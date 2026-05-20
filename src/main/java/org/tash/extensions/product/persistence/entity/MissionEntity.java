package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.CascadeType;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.OneToMany;
import jakarta.persistence.OrderBy;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Entity
@Table(name = "ops_mission")
public class MissionEntity extends BaseOperationalEntity {
    @Column(name = "mission_number", nullable = false, unique = true)
    private String missionNumber;
    @Column(name = "title")
    private String title;
    @Column(name = "status", nullable = false)
    private String status = "DRAFT";
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "locked_by")
    private String lockedBy;
    @Column(name = "locked_at")
    private ZonedDateTime lockedAt;
    @OneToMany(mappedBy = "mission", cascade = CascadeType.ALL, orphanRemoval = true)
    @OrderBy("createdAt ASC")
    private List<ReservationEntity> reservations = new ArrayList<>();

    public String getMissionNumber() { return missionNumber; }
    public void setMissionNumber(String missionNumber) { this.missionNumber = missionNumber; }
    public String getTitle() { return title; }
    public void setTitle(String title) { this.title = title; }
    public String getStatus() { return status; }
    public void setStatus(String status) { this.status = status; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getLockedBy() { return lockedBy; }
    public void setLockedBy(String lockedBy) { this.lockedBy = lockedBy; }
    public ZonedDateTime getLockedAt() { return lockedAt; }
    public void setLockedAt(ZonedDateTime lockedAt) { this.lockedAt = lockedAt; }
    public List<ReservationEntity> getReservations() { return reservations; }
    public void setReservations(List<ReservationEntity> reservations) { this.reservations = reservations; }
}
