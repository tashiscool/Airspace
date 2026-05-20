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

import java.util.ArrayList;
import java.util.List;

@Entity
@Table(name = "ops_altrv_route_group")
public class AltrvRouteGroupEntity extends BaseOperationalEntity {
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "altrv_message_id", nullable = false)
    private AltrvMessageEntity altrvMessage;
    @Column(name = "group_name")
    private String groupName;
    @Column(name = "route_family")
    private String routeFamily;
    @Column(name = "source_text", columnDefinition = "TEXT")
    private String sourceText;
    @Column(name = "sequence_index", nullable = false)
    private int sequenceIndex;
    @OneToMany(mappedBy = "routeGroup", cascade = CascadeType.ALL, orphanRemoval = true)
    @OrderBy("sequenceIndex ASC")
    private List<AltrvRouteEntity> routes = new ArrayList<>();

    public AltrvMessageEntity getAltrvMessage() { return altrvMessage; }
    public void setAltrvMessage(AltrvMessageEntity altrvMessage) { this.altrvMessage = altrvMessage; }
    public String getGroupName() { return groupName; }
    public void setGroupName(String groupName) { this.groupName = groupName; }
    public String getRouteFamily() { return routeFamily; }
    public void setRouteFamily(String routeFamily) { this.routeFamily = routeFamily; }
    public String getSourceText() { return sourceText; }
    public void setSourceText(String sourceText) { this.sourceText = sourceText; }
    public int getSequenceIndex() { return sequenceIndex; }
    public void setSequenceIndex(int sequenceIndex) { this.sequenceIndex = sequenceIndex; }
    public List<AltrvRouteEntity> getRoutes() { return routes; }
    public void setRoutes(List<AltrvRouteEntity> routes) { this.routes = routes; }
}
