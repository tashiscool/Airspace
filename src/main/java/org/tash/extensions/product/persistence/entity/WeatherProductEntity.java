package org.tash.extensions.product.persistence.entity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.Id;
import jakarta.persistence.Table;

import java.time.ZonedDateTime;

@Entity
@Table(name = "ops_weather_product")
public class WeatherProductEntity {
    @Id
    @Column(name = "id", nullable = false)
    private String id;
    @Column(name = "product_type", nullable = false)
    private String productType;
    @Column(name = "provider")
    private String provider;
    @Column(name = "source_product")
    private String sourceProduct;
    @Column(name = "valid_start")
    private ZonedDateTime validStart;
    @Column(name = "valid_end")
    private ZonedDateTime validEnd;
    @Column(name = "confidence")
    private Double confidence;
    @Column(name = "raw_text", columnDefinition = "TEXT")
    private String rawText;
    @Column(name = "product_json", nullable = false, columnDefinition = "TEXT")
    private String productJson;
    @Column(name = "received_at", nullable = false)
    private ZonedDateTime receivedAt;

    public String getId() { return id; }
    public void setId(String id) { this.id = id; }
    public String getProductType() { return productType; }
    public void setProductType(String productType) { this.productType = productType; }
    public String getProvider() { return provider; }
    public void setProvider(String provider) { this.provider = provider; }
    public String getSourceProduct() { return sourceProduct; }
    public void setSourceProduct(String sourceProduct) { this.sourceProduct = sourceProduct; }
    public ZonedDateTime getValidStart() { return validStart; }
    public void setValidStart(ZonedDateTime validStart) { this.validStart = validStart; }
    public ZonedDateTime getValidEnd() { return validEnd; }
    public void setValidEnd(ZonedDateTime validEnd) { this.validEnd = validEnd; }
    public Double getConfidence() { return confidence; }
    public void setConfidence(Double confidence) { this.confidence = confidence; }
    public String getRawText() { return rawText; }
    public void setRawText(String rawText) { this.rawText = rawText; }
    public String getProductJson() { return productJson; }
    public void setProductJson(String productJson) { this.productJson = productJson; }
    public ZonedDateTime getReceivedAt() { return receivedAt; }
    public void setReceivedAt(ZonedDateTime receivedAt) { this.receivedAt = receivedAt; }
}
