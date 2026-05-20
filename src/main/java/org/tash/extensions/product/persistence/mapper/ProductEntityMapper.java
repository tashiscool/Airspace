package org.tash.extensions.product.persistence.mapper;

import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.product.persistence.entity.OperationalDecisionEntity;
import org.tash.extensions.product.persistence.entity.WeatherProductEntity;
import org.tash.extensions.weather.product.WeatherProduct;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;

public class ProductEntityMapper {
    public WeatherProductEntity toEntity(WeatherProduct product) {
        WeatherProductEntity entity = new WeatherProductEntity();
        entity.setId(product.getId());
        entity.setProductType(product.getType() == null ? "UNKNOWN" : product.getType().name());
        entity.setProvider(product.getProvider());
        entity.setSourceProduct(product.getSourceProduct());
        entity.setValidStart(product.getValidity() == null ? null : product.getValidity().getValidStart());
        entity.setValidEnd(product.getValidity() == null ? null : product.getValidity().getValidEnd());
        entity.setConfidence(product.confidenceValue());
        entity.setRawText(product.getRawText());
        entity.setProductJson(CanonicalJson.write(product));
        entity.setReceivedAt(product.getReceivedAt() == null ? ZonedDateTime.now(ZoneOffset.UTC) : product.getReceivedAt());
        return entity;
    }

    public OperationalDecisionEntity toEntity(OperationalDecisionResult result) {
        OperationalDecisionEntity entity = new OperationalDecisionEntity();
        entity.setAction(result.getAction() == null ? "UNKNOWN" : result.getAction().name());
        entity.setRecommendedAction(result.getRecommendedAction() == null ? null : result.getRecommendedAction().name());
        entity.setConfidence(result.getConfidence());
        entity.setRationale(result.getRationale());
        entity.setResultJson(CanonicalJson.write(result));
        entity.setAuditJson(result.getAuditEnvelope() == null ? null : CanonicalJson.write(result.getAuditEnvelope()));
        entity.setReplayJson(result.getReplayBundle() == null ? null : CanonicalJson.write(result.getReplayBundle()));
        return entity;
    }
}
