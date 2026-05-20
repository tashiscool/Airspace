package org.tash.extensions.carf.refdata;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class CarfSchemaCatalog {
    private final Map<String, CarfSchemaTable> tables;

    public CarfSchemaCatalog() {
        this.tables = buildTables();
    }

    public List<String> coreTrainingBackupTables() {
        return Collections.unmodifiableList(tables.values().stream()
                .map(CarfSchemaTable::getTableName)
                .collect(Collectors.toList()));
    }

    public Collection<CarfSchemaTable> tables() {
        return Collections.unmodifiableCollection(tables.values());
    }

    public Optional<CarfSchemaTable> table(String tableName) {
        return Optional.ofNullable(tables.get(normalize(tableName)));
    }

    public List<CarfSchemaTable> tablesByCategory(CarfSchemaCategory category) {
        return tables.values().stream()
                .filter(table -> table.getCategory() == category)
                .collect(Collectors.toList());
    }

    public List<CarfSchemaTable> tablesByUse(CarfSchemaUse use) {
        return tables.values().stream()
                .filter(table -> table.getUse() == use)
                .collect(Collectors.toList());
    }

    public String domainMappingFor(String tableName) {
        return table(tableName)
                .map(CarfSchemaTable::getDomainType)
                .orElse("reference-only");
    }

    public CarfSchemaCategory categoryFor(String tableName) {
        return table(tableName)
                .map(CarfSchemaTable::getCategory)
                .orElse(CarfSchemaCategory.UNKNOWN);
    }

    public CarfSchemaUse useFor(String tableName) {
        return table(tableName)
                .map(CarfSchemaTable::getUse)
                .orElse(CarfSchemaUse.REFERENCE_ONLY);
    }

    public boolean isKnownTable(String tableName) {
        return tables.containsKey(normalize(tableName));
    }

    private Map<String, CarfSchemaTable> buildTables() {
        Map<String, CarfSchemaTable> result = new LinkedHashMap<>();

        add(result, table("t_Message", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.ADAPTER_TARGET,
                "CarfMessageEnvelope", "Base persisted message envelope for CARF/USNS traffic.",
                pk("MESSAGEID", "ID"), parents(), children("t_ALTRVMessage", "t_CarfMessage", "t_CancelMessage", "t_EditMessage"),
                cols("ACCOUNT", "ORIGIN", "RECEIVEDDATE", "TEXT")));
        add(result, table("t_ALTRVMessage", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.IMPLEMENTED,
                "AltrvMessage", "Typed ALTRV A-G message root used by the modern parser and mapper.",
                pk("MESSAGEID", "ALTRVMESSAGEID", "ID"), parents("t_Message", "t_Mission"),
                children("t_DepartingGroup", "t_RouteGroup", "t_Subsection_F", "t_Subsection_G", "t_StationaryReservation"),
                cols("ACTIVITYNAME", "AVANA", "STATUS", "RAW_TEXT")));
        add(result, table("t_CarfMessage", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.ADAPTER_TARGET,
                "CarfAnalysisResult", "Operational CARF message record tying parser output to mission state.",
                pk("CARFMESSAGEID", "MESSAGEID", "ID"), parents("t_Message", "t_Mission"),
                children("t_Reservation"), cols("STATUS", "TYPE", "CREATEDDATE")));
        add(result, table("t_CancelMessage", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.ADAPTER_TARGET,
                "CarfLifecycleCommand", "Cancel command message; retained as a lifecycle adapter target.",
                pk("CANCELMESSAGEID", "MESSAGEID", "ID"), parents("t_Message"), children(), cols("MISSIONID", "REASON")));
        add(result, table("t_EditMessage", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.ADAPTER_TARGET,
                "CarfLifecycleCommand", "Edit/update command message; retained as a lifecycle adapter target.",
                pk("EDITMESSAGEID", "MESSAGEID", "ID"), parents("t_Message"), children(), cols("MISSIONID", "REVISION")));
        add(result, table("t_Mission", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.IMPLEMENTED,
                "CarfAnalysisResult", "Mission aggregate containing messages, reservations, approvals, and lifecycle state.",
                pk("MISSIONID", "ID"), parents(), children("t_ALTRVMessage", "t_Reservation", "t_Approval", "t_APREQ"),
                cols("STATUS", "LOCKEDBY", "LOCKEDDATE", "STARTDATE", "ENDDATE"), true, false));
        add(result, table("t_ProcessedMessage", CarfSchemaCategory.MESSAGE_LIFECYCLE, CarfSchemaUse.REFERENCE_ONLY,
                "ProcessedMessageAudit", "Legacy audit marker for completed inbound processing.",
                pk("PROCESSEDMESSAGEID", "MESSAGEID", "ID"), parents("t_Message"), children(), cols("PROCESSEDDATE", "RESULT")));

        add(result, table("t_DepartingGroup", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvDepartureGroup", "A/B section aircraft and callsign grouping.",
                pk("DEPARTINGGROUPID", "ID"), parents("t_ALTRVMessage"), children("t_CallSign", "t_AircraftType", "t_Departure"),
                cols("GROUPINDEX", "ADMIS", "FORMATION")));
        add(result, table("t_CallSign", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvCallsign", "Callsign values and validation targets.",
                pk("CALLSIGNID", "ID"), parents("t_DepartingGroup"), children(), cols("CALLSIGN", "SEQUENCE")));
        add(result, table("t_AircraftType", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvAircraft", "Aircraft type/count data from B section.",
                pk("AIRCRAFTTYPEID", "ID"), parents("t_DepartingGroup"), children(), cols("TYPE", "COUNT")));
        add(result, table("t_Departure", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvDeparture", "Departure location and ETD timing from C/F sections.",
                pk("DEPARTUREID", "ID"), parents("t_DepartingGroup", "t_Subsection_F"), children(),
                cols("LOCATION", "ETD", "CALLSIGN", "AVANA")));
        add(result, table("t_Subsection_F", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvTimingSection", "F section departure, AVANA, and callsign timing data.",
                pk("SUBSECTIONFID", "ID"), parents("t_ALTRVMessage"), children("t_Departure"),
                cols("ETD", "AVANA", "CALLSIGN")));
        add(result, table("t_Subsection_G", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvRemarksSection", "G section TAS/project officer/remarks and parser metadata.",
                pk("SUBSECTIONGID", "ID"), parents("t_ALTRVMessage"), children(), cols("TAS", "REMARKS", "ARTCCS")));
        add(result, table("t_Comments", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.ADAPTER_TARGET,
                "AltrvComment", "Free-text comments preserved for diagnostics/display.",
                pk("COMMENTID", "ID"), parents("t_ALTRVMessage", "t_RouteEvent"), children(), cols("TEXT", "SECTION")));
        add(result, table("t_Duration", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltrvDuration", "Duration/admission/level-off timing value object.",
                pk("DURATIONID", "ID"), parents("t_RouteEvent"), children(), cols("MINUTES", "SECONDS", "RAW")));
        add(result, table("t_Range", CarfSchemaCategory.ALTRV_STRUCTURE, CarfSchemaUse.IMPLEMENTED,
                "AltitudeRange", "Flight-level/altitude range persistence target.",
                pk("RANGEID", "ID"), parents("t_RouteEvent", "t_Reservation"), children(), cols("LOWER", "UPPER", "UNIT")));

        add(result, table("t_RouteGroup", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvRouteGroup", "D section route group and graph root.",
                pk("ROUTEGROUPID", "ID"), parents("t_ALTRVMessage"), children("t_Route", "t_RGroupings"),
                cols("GROUPNAME", "ROUTETYPE", "SEQUENCE")));
        add(result, table("t_Route", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvRoute", "Route family: implicit, partial, common, branch, reverse, alternate departure.",
                pk("ROUTEID", "ID"), parents("t_RouteGroup"), children("t_FixTime", "t_Event", "t_RouteEvent", "t_Exit"),
                cols("ROUTENAME", "ROUTETYPE", "STARTFIX", "ENDFIX")));
        add(result, table("t_RGroupings", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.ADAPTER_TARGET,
                "AltrvRouteGraphEdge", "Join table for nested route groups/common-route membership.",
                pk("RGROUPINGID", "ID"), parents("t_RouteGroup", "t_Route"), children(), cols("PARENTID", "CHILDID", "SEQUENCE")));
        add(result, table("t_BranchRoute", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvBranchRoute", "Branch-route begin/close/merge structure.",
                pk("BRANCHROUTEID", "ROUTEID", "ID"), parents("t_Route"), children(), cols("BRANCHPOINT", "MERGEPOINT")));
        add(result, table("t_CommonEvents", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvCommonRouteEvent", "Events shared by common-route receivers.",
                pk("COMMONEVENTID", "EVENTID", "ID"), parents("t_Route", "t_Event"), children(), cols("JOINPOINT", "CALLSIGN")));
        add(result, table("t_CommonExit", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvRouteExit", "Common-route exit semantics.",
                pk("COMMONEXITID", "EXITID", "ID"), parents("t_Route", "t_Exit"), children(), cols("EXITFIX", "CALLSIGN")));
        add(result, table("t_Exit", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvRouteExit", "LEAVE/LAND/ALTRV ENDS route destination edges.",
                pk("EXITID", "ID"), parents("t_Route"), children("t_Destination"), cols("TYPE", "FIX", "CALLSIGN")));
        add(result, table("t_Destination", CarfSchemaCategory.ROUTE_GRAPH, CarfSchemaUse.IMPLEMENTED,
                "AltrvDestination", "Route destination/landing endpoint.",
                pk("DESTINATIONID", "ID"), parents("t_Exit", "t_Route"), children(), cols("LOCATION", "TYPE")));

        add(result, table("t_Event", CarfSchemaCategory.ROUTE_EVENT, CarfSchemaUse.IMPLEMENTED,
                "AltrvRouteEvent", "Generic route event: JOIN, LEAVE, LAND, AIRFL, AR, RAVEC, ENCAN/EXCAN, etc.",
                pk("EVENTID", "ID"), parents("t_Route"), children("t_Area", "t_Range", "t_Duration"),
                cols("EVENTTYPE", "FIX", "TIME", "ALTITUDE")));
        add(result, table("t_RouteEvent", CarfSchemaCategory.ROUTE_EVENT, CarfSchemaUse.IMPLEMENTED,
                "AltrvRouteEvent", "Legacy alternate table name for event rows.",
                pk("ROUTEEVENTID", "EVENTID", "ID"), parents("t_Route"), children("t_Area"),
                cols("EVENTTYPE", "SOURCE_TEXT", "SEQUENCE")));
        add(result, table("t_FixTime", CarfSchemaCategory.ROUTE_EVENT, CarfSchemaUse.IMPLEMENTED,
                "AltrvRoutePoint", "Fix plus elapsed/absolute route time and source span.",
                pk("FIXTIMEID", "FIXID", "ID"), parents("t_Route"), children("t_Fix"),
                cols("FIX", "TIME", "ELAPSED", "SEQUENCE")));
        add(result, table("t_Fix", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.IMPLEMENTED,
                "CarfWaypoint", "Inline or resolved route fix/navaid/coordinate.",
                pk("FIXID", "ID", "NAME"), parents("t_FixTime", "t_AreaFix"), children(), cols("NAME", "LATITUDE", "LONGITUDE")));
        add(result, table("t_Waypoint", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.IMPLEMENTED,
                "CarfWaypoint", "Waypoint reference abstraction for fixes and route points.",
                pk("WAYPOINTID", "FIXID", "ID", "NAME"), parents(), children(), cols("IDENTIFIER", "LATITUDE", "LONGITUDE")));

        add(result, table("t_Area", CarfSchemaCategory.SPATIAL_AREA, CarfSchemaUse.IMPLEMENTED,
                "CarfReservationEvent", "Line, polygon, circle, BNDD BY, WITHIN, maneuver, orbit, and timing area source.",
                pk("AREAID", "ID"), parents("t_Event", "t_StationaryReservation"), children("t_AreaFix", "t_AreaGroup"),
                cols("AREATYPE", "RADIUS", "WIDTH", "SOURCE_TEXT")));
        add(result, table("t_AreaFix", CarfSchemaCategory.SPATIAL_AREA, CarfSchemaUse.IMPLEMENTED,
                "CarfAreaPoint", "Ordered fix list for area boundaries and route widths.",
                pk("AREAFIXID", "ID"), parents("t_Area", "t_Fix"), children(), cols("AREAID", "FIXID", "SEQUENCE")));
        add(result, table("t_AreaGroup", CarfSchemaCategory.SPATIAL_AREA, CarfSchemaUse.ADAPTER_TARGET,
                "CarfAreaGroup", "Area grouping for common/broad-front/multi-area reservations.",
                pk("AREAGROUPID", "ID"), parents("t_Area"), children("t_Area"), cols("GROUPTYPE", "SEQUENCE")));
        add(result, table("t_CommonArea", CarfSchemaCategory.SPATIAL_AREA, CarfSchemaUse.IMPLEMENTED,
                "CarfReservationEvent", "Shared area geometry for common-route and stationary constructs.",
                pk("COMMONAREAID", "AREAID", "ID"), parents("t_Area"), children(), cols("AREAID", "ROUTEID")));
        add(result, table("t_StationaryArea", CarfSchemaCategory.SPATIAL_AREA, CarfSchemaUse.IMPLEMENTED,
                "CarfReservationEvent", "Stationary reservation area geometry.",
                pk("STATIONARYAREAID", "AREAID", "ID"), parents("t_StationaryReservation", "t_Area"), children(), cols("AREAID", "RADIUS")));
        add(result, table("t_StationaryReservation", CarfSchemaCategory.RESERVATION, CarfSchemaUse.IMPLEMENTED,
                "CarfReservationEvent", "Stationary 1./2. reservation message sections.",
                pk("STATIONARYRESERVATIONID", "RESERVATIONID", "ID"), parents("t_ALTRVMessage"), children("t_StationaryArea"),
                cols("STARTDATE", "ENDDATE", "LOWERALTITUDE", "UPPERALTITUDE")));
        add(result, table("t_Reservation", CarfSchemaCategory.RESERVATION, CarfSchemaUse.IMPLEMENTED,
                "AirspaceReservation", "Concrete deconfliction reservation volume generated from parsed CARF structures.",
                pk("RESERVATIONID", "ID"), parents("t_Mission", "t_Route", "t_Area"), children("t_Separation"),
                cols("STARTDATE", "ENDDATE", "LOWERALTITUDE", "UPPERALTITUDE", "SOURCE_RATIO")));
        add(result, table("t_Separation", CarfSchemaCategory.RESERVATION, CarfSchemaUse.IMPLEMENTED,
                "CarfSeparationStandard", "Per-reservation lateral/vertical/longitudinal separation standard.",
                pk("SEPARATIONID", "ID"), parents("t_Reservation"), children(), cols("LATERAL", "VERTICAL", "LONGITUDINAL")));

        add(result, table("t_Navaids", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.IMPLEMENTED,
                "CarfReferenceDataProvider", "Navaid/fix lookup rows for named route resolution.",
                pk("NAVAID", "NAVAIDID", "ID", "FIXID"), parents(), children("t_Fix"), cols("IDENT", "LATITUDE", "LONGITUDE", "COUNTRY")));
        add(result, table("t_PreferedNavaids", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.IMPLEMENTED,
                "CarfReferenceDataProvider", "Preferred duplicate-navaid selection rules.",
                pk("NAVAID", "NAVAIDID", "ID", "FIXID"), parents("t_Navaids"), children(), cols("IDENT", "PRIORITY", "COUNTRY")));
        add(result, table("t_LocId", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.ADAPTER_TARGET,
                "LocationReference", "Location/account identifier reference data.",
                pk("LOCID", "ID", "LOCATION"), parents(), children(), cols("LOCATION", "FACILITY", "FIR")));
        add(result, table("t_ARTCC", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.ADAPTER_TARGET,
                "ArtccReference", "ARTCC/FIR facility reference rows.",
                pk("ARTCCID", "ID", "IDENTIFIER"), parents(), children(), cols("IDENTIFIER", "NAME")));
        add(result, table("t_FromAddress", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.ADAPTER_TARGET,
                "MessageAddressReference", "Allowed origin/from-address records.",
                pk("FROMADDRESSID", "ID", "ADDRESS"), parents(), children(), cols("ADDRESS", "ACCOUNT")));
        add(result, table("t_Recipient", CarfSchemaCategory.REFERENCE_DATA, CarfSchemaUse.ADAPTER_TARGET,
                "MessageAddressReference", "Outbound recipient/address records.",
                pk("RECIPIENTID", "ID", "ADDRESS"), parents(), children(), cols("ADDRESS", "TYPE")));

        add(result, table("t_Notams", CarfSchemaCategory.NOTAM, CarfSchemaUse.ADAPTER_TARGET,
                "NotamParseResult", "NOTAM references tied to CARF lifecycle and USNS ingestion.",
                pk("NOTAMID", "ID"), parents("t_Mission"), children(), cols("ACCOUNT", "NUMBER", "TEXT", "STATUS")));
        add(result, table("t_APREQ", CarfSchemaCategory.OPERATIONAL_AUDIT, CarfSchemaUse.ADAPTER_TARGET,
                "ApreqLifecycleRecord", "Approval request coordination record.",
                pk("APREQID", "ID"), parents("t_Mission"), children(), cols("STATUS", "REQUESTEDDATE")));
        add(result, table("t_Approval", CarfSchemaCategory.OPERATIONAL_AUDIT, CarfSchemaUse.ADAPTER_TARGET,
                "ApprovalRecord", "Mission/reservation approval state.",
                pk("APPROVALID", "ID"), parents("t_Mission"), children(), cols("APPROVEDBY", "APPROVEDDATE", "STATUS")));
        add(result, table("t_Validation", CarfSchemaCategory.OPERATIONAL_AUDIT, CarfSchemaUse.IMPLEMENTED,
                "AltrvDiagnostic", "Parser/route-graph validation diagnostics.",
                pk("VALIDATIONID", "ID"), parents("t_ALTRVMessage", "t_Mission"), children(), cols("SEVERITY", "MESSAGE", "SECTION")));

        add(result, table("t_IncomingQueue", CarfSchemaCategory.QUEUE, CarfSchemaUse.ADAPTER_TARGET,
                "UsnsIngestQueueRecord", "Inbound message queue target; not hard-wired into framework execution.",
                pk("INCOMINGQUEUEID", "ID"), parents("t_Message"), children(), cols("STATUS", "RECEIVEDDATE"), true, true));
        add(result, table("t_OutgoingQueue", CarfSchemaCategory.QUEUE, CarfSchemaUse.ADAPTER_TARGET,
                "UsnsOutgoingQueueRecord", "Outbound message/report queue target.",
                pk("OUTGOINGQUEUEID", "ID"), parents("t_Message"), children(), cols("STATUS", "SENTDATE"), true, true));
        add(result, table("t_WorkQueue", CarfSchemaCategory.QUEUE, CarfSchemaUse.ADAPTER_TARGET,
                "CarfWorkQueueRecord", "Operational work queue for mission processing.",
                pk("WORKQUEUEID", "ID"), parents("t_Mission"), children(), cols("STATUS", "ASSIGNEDTO"), true, true));

        add(result, table("t_User", CarfSchemaCategory.SECURITY, CarfSchemaUse.REFERENCE_ONLY,
                "UserIdentity", "Legacy user identity; live security integration remains external.",
                pk("USERID", "ID", "USERNAME"), parents(), children("t_UserLock", "t_ScarfLock"), cols("USERNAME", "ROLE")));
        add(result, table("t_UserLock", CarfSchemaCategory.SECURITY, CarfSchemaUse.IMPLEMENTED,
                "ReservationLifecycleService", "User lock rows; stale lock release rules are modeled as pure lifecycle logic.",
                pk("USERLOCKID", "ID"), parents("t_User", "t_Mission"), children(), cols("LOCKEDBY", "LOCKEDDATE"), true, false));
        add(result, table("t_ScarfLock", CarfSchemaCategory.SECURITY, CarfSchemaUse.IMPLEMENTED,
                "ReservationLifecycleService", "SCARF mission lock rows; stale lock release rules are modeled as pure lifecycle logic.",
                pk("SCARFLOCKID", "ID"), parents("t_User", "t_Mission"), children(), cols("LOCKEDBY", "LOCKEDDATE"), true, false));
        add(result, table("t_Config", CarfSchemaCategory.CONFIGURATION, CarfSchemaUse.ADAPTER_TARGET,
                "CarfConfiguration", "Operational configuration values; loaded through adapters when needed.",
                pk("CONFIGID", "ID", "KEY"), parents(), children(), cols("KEY", "VALUE", "SCOPE")));

        return Collections.unmodifiableMap(result);
    }

    private void add(Map<String, CarfSchemaTable> result, CarfSchemaTable table) {
        result.put(normalize(table.getTableName()), table);
    }

    private CarfSchemaTable table(String name,
                                  CarfSchemaCategory category,
                                  CarfSchemaUse use,
                                  String domainType,
                                  String summary,
                                  List<String> primaryKeys,
                                  List<String> parents,
                                  List<String> children,
                                  List<String> columns) {
        return table(name, category, use, domainType, summary, primaryKeys, parents, children, columns, false, false);
    }

    private CarfSchemaTable table(String name,
                                  CarfSchemaCategory category,
                                  CarfSchemaUse use,
                                  String domainType,
                                  String summary,
                                  List<String> primaryKeys,
                                  List<String> parents,
                                  List<String> children,
                                  List<String> columns,
                                  boolean operationalMutable,
                                  boolean queueTable) {
        return CarfSchemaTable.builder()
                .tableName(name)
                .category(category)
                .use(use)
                .domainType(domainType)
                .summary(summary)
                .primaryKeyColumns(primaryKeys)
                .parentTables(parents)
                .childTables(children)
                .representativeColumns(columns)
                .operationalMutable(operationalMutable)
                .queueTable(queueTable)
                .build();
    }

    private List<String> pk(String... values) {
        return Arrays.asList(values);
    }

    private List<String> parents(String... values) {
        return Arrays.asList(values);
    }

    private List<String> children(String... values) {
        return Arrays.asList(values);
    }

    private List<String> cols(String... values) {
        return Arrays.asList(values);
    }

    private String normalize(String tableName) {
        return tableName == null ? "" : tableName.trim().toLowerCase(Locale.US);
    }
}
