package org.tash.extensions.carf.refdata;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

public class CarfSchemaRowMapper {
    private final CarfSchemaCatalog catalog;

    public CarfSchemaRowMapper() {
        this(new CarfSchemaCatalog());
    }

    public CarfSchemaRowMapper(CarfSchemaCatalog catalog) {
        this.catalog = catalog;
    }

    public CarfSchemaRow map(String tableName, Map<String, String> row) {
        Map<String, String> normalized = normalize(row);
        CarfSchemaTable table = catalog.table(tableName).orElse(null);
        return CarfSchemaRow.builder()
                .tableName(tableName)
                .knownTable(table != null)
                .category(table == null ? CarfSchemaCategory.UNKNOWN : table.getCategory())
                .use(table == null ? CarfSchemaUse.REFERENCE_ONLY : table.getUse())
                .domainType(catalog.domainMappingFor(tableName))
                .primaryId(primaryId(tableName, normalized, table))
                .fields(Collections.unmodifiableMap(normalized))
                .build();
    }

    private String primaryId(String tableName, Map<String, String> row, CarfSchemaTable table) {
        if (table != null) {
            String fromSchema = first(row, table.getPrimaryKeyColumns().toArray(new String[0]));
            if (!fromSchema.isEmpty()) {
                return fromSchema;
            }
        }
        String normalizedTable = tableName == null ? "" : tableName.trim();
        if ("t_Navaids".equals(normalizedTable) || "t_PreferedNavaids".equals(normalizedTable)) {
            return first(row, "NAVAID", "NAVAIDID", "ID", "FIXID");
        }
        if ("t_ALTRVMessage".equals(normalizedTable) || "t_Message".equals(normalizedTable)) {
            return first(row, "MESSAGEID", "ID", "MSGID");
        }
        if ("t_Route".equals(normalizedTable) || "t_RouteGroup".equals(normalizedTable)) {
            return first(row, "ROUTEID", "ROUTEGROUPID", "ID");
        }
        if ("t_Event".equals(normalizedTable) || "t_RouteEvent".equals(normalizedTable)) {
            return first(row, "EVENTID", "ID");
        }
        if ("t_FixTime".equals(normalizedTable) || "t_Fix".equals(normalizedTable) || "t_Waypoint".equals(normalizedTable)) {
            return first(row, "FIXID", "WAYPOINTID", "ID", "NAME");
        }
        if ("t_Area".equals(normalizedTable) || "t_StationaryArea".equals(normalizedTable)
                || "t_StationaryReservation".equals(normalizedTable)) {
            return first(row, "AREAID", "RESERVATIONID", "ID");
        }
        if ("t_Reservation".equals(normalizedTable) || "t_Mission".equals(normalizedTable)) {
            return first(row, "RESERVATIONID", "MISSIONID", "ID");
        }
        return first(row, "ID", "NAME", "KEY");
    }

    private String first(Map<String, String> row, String... names) {
        for (String name : names) {
            String value = row.get(name);
            if (value != null && !value.trim().isEmpty()) {
                return value;
            }
        }
        return "";
    }

    private Map<String, String> normalize(Map<String, String> row) {
        if (row == null) {
            return Collections.emptyMap();
        }
        Map<String, String> normalized = new LinkedHashMap<>();
        for (Map.Entry<String, String> entry : row.entrySet()) {
            normalized.put(entry.getKey() == null ? "" : entry.getKey().trim().toUpperCase(Locale.US),
                    entry.getValue());
        }
        return normalized;
    }
}
