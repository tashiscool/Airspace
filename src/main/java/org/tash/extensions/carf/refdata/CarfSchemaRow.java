package org.tash.extensions.carf.refdata;

import lombok.Builder;
import lombok.Data;

import java.util.Map;

@Data
@Builder
public class CarfSchemaRow {
    private String tableName;
    private String domainType;
    private String primaryId;
    private Map<String, String> fields;
}
