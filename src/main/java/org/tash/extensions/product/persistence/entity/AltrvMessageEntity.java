package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.CascadeType;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.OneToMany;
import jakarta.persistence.OrderBy;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Entity
@Table(name = "ops_altrv_message")
public class AltrvMessageEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "mission_id")
    private MissionEntity mission;
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "reservation_id")
    private ReservationEntity reservation;
    @Column(name = "activity_name")
    private String activityName;
    @Column(name = "message_type")
    private String messageType;
    @Column(name = "parser_status", nullable = false)
    private String parserStatus = "UNPARSED";
    @Column(name = "avana_at")
    private ZonedDateTime avanaAt;
    @Column(name = "tas")
    private String tas;
    @Column(name = "project_officer")
    private String projectOfficer;
    @Column(name = "alternate_project_officer")
    private String alternateProjectOfficer;
    @Column(name = "artccs_concerned", columnDefinition = "TEXT")
    private String artccsConcerned;
    @Column(name = "additional_info", columnDefinition = "TEXT")
    private String additionalInfo;
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "diagnostics_json", columnDefinition = "TEXT")
    private String diagnosticsJson;
    @Column(name = "parsed_json", columnDefinition = "TEXT")
    private String parsedJson;
    @OneToMany(mappedBy = "altrvMessage", cascade = CascadeType.ALL, orphanRemoval = true)
    @OrderBy("sequenceIndex ASC")
    private List<AltrvRouteGroupEntity> routeGroups = new ArrayList<>();

    public MissionEntity getMission() { return mission; }
    public void setMission(MissionEntity mission) { this.mission = mission; }
    public ReservationEntity getReservation() { return reservation; }
    public void setReservation(ReservationEntity reservation) { this.reservation = reservation; }
    public String getActivityName() { return activityName; }
    public void setActivityName(String activityName) { this.activityName = activityName; }
    public String getMessageType() { return messageType; }
    public void setMessageType(String messageType) { this.messageType = messageType; }
    public String getParserStatus() { return parserStatus; }
    public void setParserStatus(String parserStatus) { this.parserStatus = parserStatus; }
    public ZonedDateTime getAvanaAt() { return avanaAt; }
    public void setAvanaAt(ZonedDateTime avanaAt) { this.avanaAt = avanaAt; }
    public String getTas() { return tas; }
    public void setTas(String tas) { this.tas = tas; }
    public String getProjectOfficer() { return projectOfficer; }
    public void setProjectOfficer(String projectOfficer) { this.projectOfficer = projectOfficer; }
    public String getAlternateProjectOfficer() { return alternateProjectOfficer; }
    public void setAlternateProjectOfficer(String alternateProjectOfficer) { this.alternateProjectOfficer = alternateProjectOfficer; }
    public String getArtccsConcerned() { return artccsConcerned; }
    public void setArtccsConcerned(String artccsConcerned) { this.artccsConcerned = artccsConcerned; }
    public String getAdditionalInfo() { return additionalInfo; }
    public void setAdditionalInfo(String additionalInfo) { this.additionalInfo = additionalInfo; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getDiagnosticsJson() { return diagnosticsJson; }
    public void setDiagnosticsJson(String diagnosticsJson) { this.diagnosticsJson = diagnosticsJson; }
    public String getParsedJson() { return parsedJson; }
    public void setParsedJson(String parsedJson) { this.parsedJson = parsedJson; }
    public List<AltrvRouteGroupEntity> getRouteGroups() { return routeGroups; }
    public void setRouteGroups(List<AltrvRouteGroupEntity> routeGroups) { this.routeGroups = routeGroups; }
}
