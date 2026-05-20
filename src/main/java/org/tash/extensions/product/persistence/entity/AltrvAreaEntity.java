package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;

@Entity
@Table(name = "ops_altrv_area")
public class AltrvAreaEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "altrv_message_id")
    private AltrvMessageEntity altrvMessage;
    @Column(name = "area_type", nullable = false)
    private String areaType;
    @Column(name = "width_nm")
    private Double widthNm;
    @Column(name = "radius_nm")
    private Double radiusNm;
    @Column(name = "center_identifier")
    private String centerIdentifier;
    @Column(name = "source_text", columnDefinition = "TEXT")
    private String sourceText;
    @Column(name = "geometry_json", columnDefinition = "TEXT")
    private String geometryJson;
    @Column(name = "sequence_index", nullable = false)
    private int sequenceIndex;

    public AltrvMessageEntity getAltrvMessage() { return altrvMessage; }
    public void setAltrvMessage(AltrvMessageEntity altrvMessage) { this.altrvMessage = altrvMessage; }
    public String getAreaType() { return areaType; }
    public void setAreaType(String areaType) { this.areaType = areaType; }
    public Double getWidthNm() { return widthNm; }
    public void setWidthNm(Double widthNm) { this.widthNm = widthNm; }
    public Double getRadiusNm() { return radiusNm; }
    public void setRadiusNm(Double radiusNm) { this.radiusNm = radiusNm; }
    public String getCenterIdentifier() { return centerIdentifier; }
    public void setCenterIdentifier(String centerIdentifier) { this.centerIdentifier = centerIdentifier; }
    public String getSourceText() { return sourceText; }
    public void setSourceText(String sourceText) { this.sourceText = sourceText; }
    public String getGeometryJson() { return geometryJson; }
    public void setGeometryJson(String geometryJson) { this.geometryJson = geometryJson; }
    public int getSequenceIndex() { return sequenceIndex; }
    public void setSequenceIndex(int sequenceIndex) { this.sequenceIndex = sequenceIndex; }
}
