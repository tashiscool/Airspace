package org.tash.extensions.carf.refdata;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CarfSchemaCatalog {
    public List<String> coreTrainingBackupTables() {
        return Collections.unmodifiableList(Arrays.asList(
                "t_ALTRVMessage",
                "t_AircraftType",
                "t_Approval",
                "t_APREQ",
                "t_Area",
                "t_AreaFix",
                "t_AreaGroup",
                "t_ARTCC",
                "t_BranchRoute",
                "t_CallSign",
                "t_CancelMessage",
                "t_CarfMessage",
                "t_Comments",
                "t_CommonArea",
                "t_CommonEvents",
                "t_CommonExit",
                "t_Config",
                "t_DepartingGroup",
                "t_Departure",
                "t_Destination",
                "t_Duration",
                "t_EditMessage",
                "t_Event",
                "t_Exit",
                "t_Fix",
                "t_FixTime",
                "t_FromAddress",
                "t_IncomingQueue",
                "t_LocId",
                "t_Message",
                "t_Mission",
                "t_Navaids",
                "t_Notams",
                "t_OutgoingQueue",
                "t_PreferedNavaids",
                "t_ProcessedMessage",
                "t_Range",
                "t_Recipient",
                "t_Reservation",
                "t_RGroupings",
                "t_Route",
                "t_RouteGroup",
                "t_RouteEvent",
                "t_ScarfLock",
                "t_Separation",
                "t_StationaryArea",
                "t_StationaryReservation",
                "t_Subsection_F",
                "t_Subsection_G",
                "t_User",
                "t_UserLock",
                "t_Validation",
                "t_Waypoint",
                "t_WorkQueue"));
    }

    public String domainMappingFor(String tableName) {
        if (tableName == null) {
            return "unknown";
        }
        switch (tableName) {
            case "t_ALTRVMessage":
                return "AltrvMessage";
            case "t_Route":
                return "AltrvRoute";
            case "t_RouteGroup":
                return "AltrvRouteGroup";
            case "t_Event":
                return "AltrvRouteEvent";
            case "t_FixTime":
                return "AltrvRoutePoint";
            case "t_Area":
            case "t_StationaryReservation":
                return "CarfReservationEvent";
            case "t_Navaids":
            case "t_PreferedNavaids":
                return "CarfReferenceDataProvider";
            case "t_Reservation":
                return "AirspaceReservation";
            case "t_Mission":
                return "CarfAnalysisResult";
            default:
                return "reference-only";
        }
    }
}
