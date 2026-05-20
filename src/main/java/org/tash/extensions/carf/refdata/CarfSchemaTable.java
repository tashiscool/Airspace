package org.tash.extensions.carf.refdata;

import lombok.Builder;
import lombok.Data;
import lombok.Singular;

import java.util.List;

@Data
@Builder
public class CarfSchemaTable {
    private String tableName;
    private CarfSchemaCategory category;
    private CarfSchemaUse use;
    private String domainType;
    private String summary;
    @Singular
    private List<String> primaryKeyColumns;
    @Singular
    private List<String> parentTables;
    @Singular
    private List<String> childTables;
    @Singular
    private List<String> representativeColumns;
    private boolean operationalMutable;
    private boolean queueTable;
}
