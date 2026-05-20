package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.carf.altrv.AltrvParser;
import org.tash.extensions.carf.altrv.AltrvRouteEventType;
import org.tash.extensions.carf.altrv.AltrvRouteGraph;
import org.tash.extensions.carf.altrv.AltrvRouteGraphBuilder;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidation;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidator;
import org.tash.extensions.carf.altrv.AltrvRouteGraphVertexType;
import org.tash.extensions.carf.altrv.AltrvSpatialMapper;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.carf.refdata.CarfSchemaRow;
import org.tash.extensions.carf.refdata.CarfSchemaRowMapper;
import org.tash.extensions.carf.refdata.InMemoryCarfReferenceDataProvider;
import org.tash.extensions.messaging.CarfMessageFamilyParseResult;
import org.tash.extensions.messaging.CarfMessageFamilyParser;
import org.tash.extensions.messaging.CarfMessageFamilyStatus;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.messaging.MrsFunctionCode;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.messaging.UsnsRoutingOutcomeType;
import org.tash.extensions.messaging.transaction.ServiceRequestCommand;
import org.tash.extensions.messaging.transaction.UsnsTransaction;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;
import org.tash.extensions.notam.access.InMemoryNotamAccessReferenceData;
import org.tash.extensions.notam.access.NotamAccessPolicy;
import org.tash.extensions.notam.access.NotamAccessRequest;
import org.tash.extensions.notam.access.NotamAccessResult;
import org.tash.extensions.reservation.CarfReservationEvent;
import org.tash.extensions.reservation.CarfReservationEventType;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;

import static org.junit.jupiter.api.Assertions.*;

class FullGapClosureTest {
    @Test
    void parsesMrsFunctionCodesAndAppliesLegacyFdcPolicies() {
        assertEquals(MrsFunctionCode.NOTAM_TRAFFIC, MrsFunctionCode.from("01"));
        assertEquals(MrsFunctionCode.FDC_COMMENTS_EDIT, MrsFunctionCode.from("17"));
        assertEquals(MrsFunctionCode.PRIVILEGED_REQUEST, MrsFunctionCode.from("30"));
        assertEquals(MrsFunctionCode.UNKNOWN, MrsFunctionCode.from("99"));

        UsnsIngestResult wrongOrigin = new UsnsIngestService().parse(envelope("01GGNC07GP", "300334 KGPS",
                "!FDC 0/1234 ZNY AIRBORNE TO GROUND LASER ACTIVITY"));
        assertFalse(wrongOrigin.getTransactionIngestResults().get(0).getRoutingOutcome().isAccepted());
        assertEquals(UsnsRoutingOutcomeType.BYPASS_ERQ,
                wrongOrigin.getTransactionIngestResults().get(0).getRoutingOutcome().getType());

        UsnsIngestResult edit = new UsnsIngestService().parse(envelope("07GGNC07GP", "300334 KDZZNAXX",
                "!FDC 0/1234 ZNY AIRBORNE TO GROUND LASER ACTIVITY"));
        assertEquals(UsnsRoutingOutcomeType.RETURN_TO_SENDER,
                edit.getTransactionIngestResults().get(0).getRoutingOutcome().getType());

        UsnsIngestResult commentEdit = new UsnsIngestService().parse(envelope("17GGNC07GP", "300334 KDZZNAXX",
                "!FDC 0/1234 ZNY AIRBORNE TO GROUND LASER ACTIVITY"));
        assertTrue(commentEdit.getTransactionIngestResults().get(0).getRoutingOutcome().isAccepted());
        assertTrue(commentEdit.getTransactionIngestResults().get(0).getWarnings().stream()
                .anyMatch(w -> w.contains("comment edit")));
    }

    @Test
    void parsesRicherServiceRequestsAndTypedFamilySemantics() {
        ServiceRequestCommand rqn = ServiceRequestCommand.parse("RQN PDT A1234/10 B1234/10 HIST COUNT");
        assertTrue(rqn.isAccepted(), rqn.getErrors().toString());
        assertEquals("RQN", rqn.getRequestFormat());
        assertEquals("PDT", rqn.getLocation());
        assertTrue(rqn.isHistory());
        assertTrue(rqn.isCount());
        assertTrue(rqn.isPrivilegedHistoryRequest());

        CarfMessageFamilyParseResult snowtam = new CarfMessageFamilyParser().parse(UsnsTransaction.builder()
                .type(UsnsTransactionType.SNOWTAM)
                .rawText("SNOWTAM 0123")
                .build());
        CarfMessageFamilyParseResult genot = new CarfMessageFamilyParser().parse(UsnsTransaction.builder()
                .type(UsnsTransactionType.GENOT)
                .rawText("GENOT RWA ABC")
                .build());

        assertEquals(CarfMessageFamilyStatus.SUPPORTED, snowtam.getStatus());
        assertEquals("snowtam-replaces-prior-active-record", snowtam.getSemantic());
        assertEquals(CarfMessageFamilyStatus.SUPPORTED, genot.getStatus());
        assertEquals("genot-admin-message", genot.getSemantic());
    }

    @Test
    void accessPolicyRequiresAllLocationsUnlessAnyPrivilegeApplies() {
        HashSet<String> locationPrivileges = new HashSet<>(Arrays.asList(
                InMemoryNotamAccessReferenceData.key("EDIT", "JFK", "A", "D"),
                InMemoryNotamAccessReferenceData.key("EDIT", "LGA", "A", "D")));
        NotamAccessPolicy policy = new NotamAccessPolicy(new InMemoryNotamAccessReferenceData(
                Collections.emptySet(), locationPrivileges, Collections.emptySet()));

        NotamAccessResult allowed = policy.verify(NotamAccessRequest.builder()
                .privilege("EDIT")
                .series("A")
                .source("D")
                .locationIds(Arrays.asList("JFK", "LGA"))
                .build());
        NotamAccessResult denied = policy.verify(NotamAccessRequest.builder()
                .privilege("EDIT")
                .series("A")
                .source("D")
                .locationIds(Arrays.asList("JFK", "EWR"))
                .build());

        assertTrue(allowed.isAllowed(), allowed.getErrors().toString());
        assertFalse(denied.isAllowed());
        assertTrue(denied.getErrors().stream().anyMatch(e -> e.contains("EWR")));
    }

    @Test
    void altrvParserAndSpatialMapperPreserveRouteEventAndReferenceDataGaps() {
        String message = "A. TEST01\n"
                + "B. 1/F16\n"
                + "C. LOCATION\n"
                + "D. FL240B260 FIXA 0000 CLMB FL260B280 FIXB 0030 LVLOF BY 0040 JOIN CMN RTE BLUE RCVR CMN RTE CLOSE BRANCH MERGE "
                + "ALT DEP FIXC 045/010 0100 BROAD FRONT SUPERSONIC END SUPERSONIC\n"
                + "F. TEST01 ETD 302300 MAR 2010 AVANA 010030\n"
                + "G. TAS: 480 KTAS";
        Map<String, GeoCoordinate> fixes = new HashMap<>();
        fixes.put("FIXA", point(30, -150));
        fixes.put("FIXB", point(31, -151));
        fixes.put("FIXC", point(32, -152));
        CarfReferenceDataProvider refs = new InMemoryCarfReferenceDataProvider(fixes,
                Collections.singletonMap("GLOBAL", "RWY"));

        AltrvParseResult result = new AltrvParser().parse(message);
        List<CarfReservationEvent> events = new AltrvSpatialMapper().toReservationEvents(result.getMessage(), refs);
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(result.getMessage());

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.JOIN_COMMON_ROUTE));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.RECEIVER_COMMON_ROUTE));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.BRANCH_CLOSE));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.BRANCH_MERGE));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.ALTERNATE_DEPARTURE_ROUTE));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.RADIAL_DME));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.CLIMB));
        assertTrue(result.getMessage().hasEvent(AltrvRouteEventType.LEVEL_OFF));
        assertTrue(result.getMessage().getAvanaTime().isAfter(result.getMessage().getFirstDepartureTime()));
        assertTrue(events.stream().anyMatch(e -> e.getType() == CarfReservationEventType.ROUTE_SEGMENT));
        assertTrue(events.stream().anyMatch(e -> e.getSourceFixes().contains("FIXA")));
        assertTrue(events.stream()
                .filter(e -> e.getType() == CarfReservationEventType.ROUTE_SEGMENT)
                .flatMap(e -> e.getPoints().stream())
                .anyMatch(p -> point(32, -152).distanceTo(p) > 9 && point(32, -152).distanceTo(p) < 11));
        assertTrue(graph.getVertices().values().stream().anyMatch(v -> v.getType() == AltrvRouteGraphVertexType.JOIN));
        assertTrue(graph.getVertices().values().stream().anyMatch(v -> v.getType() == AltrvRouteGraphVertexType.BRANCH));
        assertEquals("RWY", refs.resolveGlobalAccount("GLOBAL").orElse(""));
    }

    @Test
    void routeGraphValidatorChecksDestinationCallsignFlow() {
        String message = "A. TEST01\n"
                + "B. 1/F16\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100\n"
                + "E. OTHER02 DEST\n"
                + "F. TEST01 ETD 021300 MAR 2010 AVANA 021400\n"
                + "G. TAS: 480 KTAS";

        AltrvParseResult result = new AltrvParser().parse(message);
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(result.getMessage());
        AltrvRouteGraphValidation validation = new AltrvRouteGraphValidator()
                .validate(result.getMessage(), graph, null);

        assertFalse(validation.isValid());
        assertTrue(validation.getDiagnostics().stream()
                .anyMatch(d -> d.contains("Destination callsign OTHER02")));
    }

    @Test
    void sectionParserReturnsSectionLevelDiagnosticsAndSchemaRowsAreMappable() {
        AltrvParseResult parsed = new org.tash.extensions.carf.altrv.AltrvSectionParser()
                .parse("A. TEST01\nD. \nG. TAS: 480 KTAS");

        assertFalse(parsed.isAccepted());
        assertTrue(parsed.getSectionResults().stream().anyMatch(section ->
                "F".equals(section.getName()) && !section.isAccepted()));
        assertTrue(parsed.getSectionResults().stream().anyMatch(section ->
                "D".equals(section.getName()) && !section.isAccepted()
                        && !section.getDiagnostics().isEmpty()));

        Map<String, String> row = new LinkedHashMap<>();
        row.put("navaidid", "DQN");
        row.put("lat", "39.755");
        row.put("lon", "-84.633");
        CarfSchemaRow mapped = new CarfSchemaRowMapper().map("t_Navaids", row);

        assertEquals("CarfReferenceDataProvider", mapped.getDomainType());
        assertEquals("DQN", mapped.getPrimaryId());
        assertEquals("-84.633", mapped.getFields().get("LON"));
    }

    private String envelope(String header, String originLine, String body) {
        return header + "\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + originLine + "\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }

    private GeoCoordinate point(double latitude, double longitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build();
    }
}
