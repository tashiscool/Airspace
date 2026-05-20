package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.evaluation.AirspaceEvaluationLedger;
import org.tash.extensions.evaluation.ArtifactEvaluationEntry;
import org.tash.extensions.evaluation.ArtifactEvaluationStatus;
import org.tash.extensions.evaluation.ArtifactEvaluator;
import org.tash.extensions.evaluation.LegacyArtifactTextExtractor;
import org.tash.extensions.evaluation.LegacyEmailAttachmentExtractor;
import org.tash.extensions.evaluation.LegacyJavaArtifactAnalyzer;
import org.tash.extensions.evaluation.LegacyRarArtifactAnalyzer;
import org.tash.extensions.evaluation.LegacySpreadsheetArtifactAnalyzer;
import org.tash.extensions.evaluation.LegacyZipArtifactExtractor;
import org.tash.extensions.evaluation.LegacyZipComparison;
import org.tash.extensions.evaluation.PostgresCustomDumpAnalyzer;
import org.tash.extensions.evaluation.VisualArtifactAnalyzer;
import org.tash.extensions.notam.FirReferenceCatalog;
import org.tash.extensions.notam.GlobalAccountKeywordResolver;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfEventReservationMapper;
import org.tash.extensions.reservation.CarfProtectedAirspaceRules;
import org.tash.extensions.reservation.CarfReservationEvent;
import org.tash.extensions.reservation.CarfReservationEventType;
import org.tash.extensions.reservation.CarfRouteMessageParser;

import java.io.ByteArrayInputStream;
import java.time.ZonedDateTime;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import static org.junit.jupiter.api.Assertions.*;

class AirspaceEvaluationHarnessTest {
    @Test
    void defaultLedgerTracksArtifactEvaluationState() {
        AirspaceEvaluationLedger ledger = AirspaceEvaluationLedger.defaultBacklog();

        assertTrue(ledger.entries().size() >= 30);
        assertTrue(ledger.count(ArtifactEvaluationStatus.TEST_ADDED) >= 12);
        assertTrue(ledger.count(ArtifactEvaluationStatus.IMPLEMENTED) >= 7);
        assertTrue(ledger.count(ArtifactEvaluationStatus.REFERENCE_ONLY) >= 8);
        assertEquals(2, ledger.count(ArtifactEvaluationStatus.BLOCKED));
        assertFalse(ledger.hasUnreviewedEntries());
        assertTrue(ledger.entries().stream().allMatch(entry -> entry.getFinding() != null && !entry.getFinding().isEmpty()));
        assertTrue(ledger.entries().stream().anyMatch(entry ->
                entry.getPath().contains("/Users/tkhan/Downloads/src 2 messaging")
                        && entry.getStatus() == ArtifactEvaluationStatus.IMPLEMENTED));
        assertTrue(ledger.entries().stream().anyMatch(entry ->
                entry.getPath().contains("/Users/tkhan/Downloads/src 2 infrastructure")
                        && entry.getStatus() == ArtifactEvaluationStatus.REFERENCE_ONLY));
    }

    @Test
    void mapsModernCarfEventsIntoReservations() {
        CarfReservationEvent event = CarfReservationEvent.builder()
                .type(CarfReservationEventType.TIMING_TRIANGLE)
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00-05:00[America/New_York]"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00-05:00[America/New_York]"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .routeWidthNauticalMiles(100)
                .points(Arrays.asList(point(30, -150), point(31, -151), point(30, -152)))
                .sourceText("legacy timing triangle")
                .build();

        List<AirspaceReservation> reservations =
                new CarfEventReservationMapper().toReservations("TRI", event);

        assertEquals(1, reservations.size());
        assertEquals("TIMING_TRIANGLE", reservations.get(0).getReservationType());
        assertEquals("legacy timing triangle", reservations.get(0).getSourceText());
        assertNotNull(reservations.get(0).getProtectedVolume());
        assertEquals(3, reservations.get(0).getProtectedVolume().getBasePolygon().getVertices().size());
        assertTrue(reservations.get(0).getProtectedVolume().getBasePolygon().getBoundingBox().getMinLat() < 30);
    }

    @Test
    void mapsRouteSegmentEventsIntoCapsuleProtectedVolumes() {
        CarfReservationEvent event = CarfReservationEvent.builder()
                .type(CarfReservationEventType.ROUTE_SEGMENT)
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .routeWidthNauticalMiles(20)
                .points(Arrays.asList(point(30, -150), point(30, -149), point(30, -148)))
                .sourceText("legacy route segment")
                .build();

        List<AirspaceReservation> reservations =
                new CarfEventReservationMapper().toReservations("ROUTE", event);

        assertEquals(2, reservations.size());
        assertTrue(reservations.stream().allMatch(r -> "ROUTE_SEGMENT".equals(r.getReservationType())));
        assertTrue(reservations.stream().allMatch(r -> r.getProtectedVolume() != null));
        assertTrue(reservations.stream().allMatch(r -> r.getProtectedVolume().getBasePolygon().getVertices().size() > 4));
    }

    @Test
    void validatesLegacyAreaAndOrbitEventRules() {
        CarfEventReservationMapper mapper = new CarfEventReservationMapper();
        CarfReservationEvent badTriangle = CarfReservationEvent.builder()
                .type(CarfReservationEventType.TIMING_TRIANGLE)
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .points(Arrays.asList(point(30, -150), point(31, -151)))
                .build();
        CarfReservationEvent orbit = CarfReservationEvent.builder()
                .type(CarfReservationEventType.ORBIT)
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .protectedRadiusNauticalMiles(25)
                .points(Arrays.asList(point(30, -150)))
                .sourceText("legacy orbit")
                .build();
        CarfReservationEvent badOrbit = CarfReservationEvent.builder()
                .type(CarfReservationEventType.ORBIT)
                .startTime(ZonedDateTime.parse("2010-03-02T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-03-02T13:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .protectedRadiusNauticalMiles(1000)
                .points(Arrays.asList(point(30, -150)))
                .build();

        assertThrows(IllegalArgumentException.class, () -> mapper.toReservations("BAD-TRI", badTriangle));
        List<AirspaceReservation> orbitReservations = mapper.toReservations("ORB", orbit);
        assertEquals(1, orbitReservations.size());
        assertEquals("ORBIT", orbitReservations.get(0).getReservationType());
        assertEquals(36, orbitReservations.get(0).getProtectedVolume().getBasePolygon().getVertices().size());
        assertTrue(orbitReservations.get(0).getProtectedVolume().contains(
                GeoCoordinate.builder().latitude(30).longitude(-150).altitude(25000).build()));
        assertThrows(IllegalArgumentException.class, () -> mapper.toReservations("BAD-ORB", badOrbit));
    }

    @Test
    void protectedAirspaceRulesReflectIrdSeparationPrinciples() {
        CarfProtectedAirspaceRules rules = new CarfProtectedAirspaceRules();

        assertTrue(rules.isSeparated(true, false, false));
        assertTrue(rules.isSeparated(false, true, false));
        assertTrue(rules.isSeparated(false, false, true));
        assertFalse(rules.isSeparated(false, false, false));
        assertEquals(23750.5, rules.deconflictionLowerAltitudeFeet(24000, 499), 0.0001);
        assertEquals(26249.5, rules.deconflictionUpperAltitudeFeet(26000, 499), 0.0001);
        assertEquals(59, rules.requiredLongitudinalMinutes(30, 59), 0.0001);
        assertTrue(rules.verticalSeparationEstablishedForPassage(
                ZonedDateTime.parse("2010-08-16T22:06:00Z"),
                ZonedDateTime.parse("2010-08-16T22:36:00Z"),
                30));
        assertFalse(rules.verticalSeparationEstablishedForPassage(
                ZonedDateTime.parse("2010-08-16T22:07:00Z"),
                ZonedDateTime.parse("2010-08-16T22:36:00Z"),
                30));
        assertEquals(29, rules.minutesVerticalSeparationEstablishedBeforePassage(
                ZonedDateTime.parse("2010-08-16T22:07:00Z"),
                ZonedDateTime.parse("2010-08-16T22:36:00Z")), 0.0001);
        assertTrue(rules.principles().stream().anyMatch(p -> p.contains("independent")));
    }

    @Test
    void modernReservationMatchesLegacyTemporalAndVerticalExpansionSemantics() {
        AirspaceReservation reservation = AirspaceReservation.builder()
                .id("LEGACY-COVERAGE")
                .startTime(ZonedDateTime.parse("2010-04-10T15:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-04-10T16:00:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .deconflictionLowerAltitudeFeet(23000)
                .deconflictionUpperAltitudeFeet(27000)
                .verticalSeparationFeet(2000)
                .avanaMinutes(60)
                .longitudinalSeparationMinutes(30)
                .routeWidthNauticalMiles(120)
                .sourceRatioStart(0)
                .sourceRatioEnd(1)
                .reservationType("ROUTE_SEGMENT")
                .routeStartFix("A")
                .routeEndFix("B")
                .sourceFixes(Arrays.asList("A", "B"))
                .sourceText("legacy source")
                .build();

        assertEquals(ZonedDateTime.parse("2010-04-10T14:30:00Z"), reservation.getEffectiveConflictStartTime());
        assertEquals(ZonedDateTime.parse("2010-04-10T17:30:00Z"), reservation.getEffectiveConflictEndTime());
        assertEquals(23000, reservation.getDeconflictionLowerAltitudeFeet(), 0.0001);
        assertEquals(27000, reservation.getDeconflictionUpperAltitudeFeet(), 0.0001);
        assertEquals("legacy source", reservation.getSourceText());
        assertEquals(Arrays.asList("A", "B"), reservation.getSourceFixes());
    }

    @Test
    void classifiesLegacyAirspaceReservationAsDesignReferenceNotCopyCandidate() throws Exception {
        LegacyJavaArtifactAnalyzer.Analysis analysis =
                new LegacyJavaArtifactAnalyzer().analyze(download("AirspaceReservation.java"));

        assertFalse(analysis.isCopyCandidate());
        assertTrue(analysis.getExternalFamilies().contains("gov.faa"));
    }

    @Test
    void detectsDuplicateForwardedArtifacts() throws Exception {
        Path first = download("conflicttoshort/90-91conflictslisting.txt");
        Path second = download("fwdconflicttoshort/90-91conflictslisting.txt");

        ArtifactEvaluationEntry entry = new ArtifactEvaluator().duplicate(first, second, "CARF duration regression");

        assertEquals(ArtifactEvaluationStatus.REFERENCE_ONLY, entry.getStatus());
        assertEquals("Duplicate artifact", entry.getFinding());
    }

    @Test
    void extractsNotamLinesFromLegacyExcelArtifacts() throws Exception {
        LegacyArtifactTextExtractor extractor = new LegacyArtifactTextExtractor();

        assertTrue(extractor.notamLines(download("AEROBAT.xls")).stream()
                .anyMatch(line -> line.contains("AEROBATIC")));
        assertTrue(extractor.notamLines(download("BERMS.xls")).stream()
                .anyMatch(line -> line.contains("BERM")));
        assertTrue(extractor.notamLines(download("PILE.xls")).stream()
                .anyMatch(line -> line.contains("PILE")));
        assertTrue(extractor.notamLines(download("REMAINING.xls")).stream()
                .anyMatch(line -> line.contains("DISTANCE REMAINING")));
        assertTrue(extractor.notamLines(download("TURNAROUND.xls")).stream()
                .anyMatch(line -> line.contains("TURNAROUND")));
    }

    @Test
    void extractsCarfTimingMessagesFromLegacyZipArtifacts() throws Exception {
        Map<String, String> entries = new LegacyZipArtifactExtractor()
                .textEntries(download("messages.zip"));

        assertTrue(entries.containsKey("timing test1.txt"));
        assertTrue(entries.containsKey("timing test 2.txt"));

        CarfRouteMessageParser parser = new CarfRouteMessageParser();
        Set<String> activityNames = new HashSet<>();
        for (String text : entries.values()) {
            activityNames.add(parser.parse(text).getActivityName());
        }
        assertTrue(activityNames.contains("WOODY01-02"));
        assertTrue(activityNames.contains("DEAL01-02"));
    }

    @Test
    void comparesLegacySourceArchivesAndIdentifiesNewerParserFiles() throws Exception {
        LegacyZipComparison.Result comparison = new LegacyZipComparison()
                .compare(download("src.zip"), download("src (2).zip"));

        assertTrue(comparison.getAdded().contains("src/FileCreator.java"));
        assertTrue(comparison.getAdded().contains("src/Main.java"));
        assertTrue(comparison.getChanged().contains("src/naimes/usns/appl/DomNotam.java"));
        assertTrue(comparison.getChanged().contains("src/naimes/usns/appl/Dom1YaccClass.java"));
        assertTrue(comparison.getChanged().contains("src/naimes/usns/appl/Dom1LexClass.java"));
    }

    @Test
    void classifiesMoreCarfWorkArchiveAsPackagedLegacyDeconflictionBundle() throws Exception {
        Map<String, String> entries = new LegacyZipArtifactExtractor()
                .textEntries(download("reenjoiitaly___buttheresmorecarfwork.zip"));

        assertTrue(entries.containsKey("ActualConflict.java"));
        assertTrue(entries.containsKey("DeconflictionContext.java"));
        assertTrue(entries.containsKey("EntityFactory.java"));
        assertTrue(entries.containsKey("TimingTriangleEvent.java"));
        assertTrue(entries.get("ActualConflict.java").contains("class ActualConflict"));
        assertTrue(entries.get("EntityFactory.java").contains("createCapsulePolygon"));
        assertTrue(entries.get("DeconflictionContext.java").contains("findAirspaceTransitionPointRatio"));
    }

    @Test
    void inventoriesParserTesterEmailAttachmentsAndWorkspaceContents() throws Exception {
        LegacyEmailAttachmentExtractor extractor = new LegacyEmailAttachmentExtractor();

        byte[] dom1 = extractor.attachment(download("Visual Parser Files.eml"), "Dom1.zip")
                .orElseThrow(() -> new AssertionError("Missing Dom1.zip"));
        byte[] visualParser = extractor.attachment(download("Visual Parser Files.eml"), "VisualParser++.rar")
                .orElseThrow(() -> new AssertionError("Missing VisualParser++.rar"));
        byte[] ibmClient = extractor.attachment(
                        download("Big Zip file for parser tester...and that's what she said.eml"),
                        "com.ibm.ws.admin.client_6.1.0.zip")
                .orElseThrow(() -> new AssertionError("Missing WebSphere client zip"));
        byte[] bundle = extractor.attachment(download("Bundle.eml"), "davorJunk.zip")
                .orElseThrow(() -> new AssertionError("Missing davorJunk.zip"));

        assertTrue(zipEntryNames(dom1).contains("Dom1/Dom1.ycc"));
        assertTrue(zipEntryNames(dom1).contains("Dom1/Dom1LexClass.java"));
        assertFalse(zipEntryNames(visualParser).isEmpty(), "VisualParser++.rar is ZIP-formatted despite its extension");
        assertEquals(new HashSet<>(Arrays.asList("com.ibm.ws.admin.client_6.1.0.jar")),
                zipEntryNames(ibmClient));

        Set<String> bundleEntries = zipEntryNames(bundle);
        assertTrue(bundleEntries.contains("davorJunk/DaveWorkspace.zip"));
        assertTrue(bundleEntries.contains("davorJunk/OriginialWorkspace.zip"));
        assertTrue(bundleEntries.contains("davorJunk/VparserInstructions.txt"));
        assertTrue(bundleEntries.contains("davorJunk/Dom1.zip"));
    }

    @Test
    void inventoriesBundleWorkspaceParserTesterFixtures() throws Exception {
        LegacyEmailAttachmentExtractor extractor = new LegacyEmailAttachmentExtractor();
        byte[] bundle = extractor.attachment(download("Bundle.eml"), "davorJunk.zip")
                .orElseThrow(() -> new AssertionError("Missing davorJunk.zip"));
        byte[] daveWorkspace = zipEntry(bundle, "davorJunk/DaveWorkspace.zip");
        byte[] originalWorkspace = zipEntry(bundle, "davorJunk/OriginialWorkspace.zip");
        String instructions = new String(zipEntry(bundle, "davorJunk/VparserInstructions.txt"), "UTF-8");

        Set<String> daveEntries = zipEntryNames(daveWorkspace);
        Set<String> originalEntries = zipEntryNames(originalWorkspace);

        assertTrue(instructions.contains("Visual Parser"));
        assertTrue(daveEntries.stream().anyMatch(name ->
                name.startsWith("DaveWorkspace/DomNotamParserTester/") && name.endsWith(".txt")));
        assertTrue(daveEntries.contains("DaveWorkspace/DomNotamParserTester/bin/naimes/usns/appl/Dom1YaccClass.class"));
        assertTrue(originalEntries.contains("OriginialWorkspace/UsnsCommon/bin/domparse2.ycc"));
        assertTrue(originalEntries.contains("OriginialWorkspace/UsnsAppClient/appClientModule/naimes/usns/client/Main.java"));
    }

    @Test
    void everyMoreCarfWorkSourceFileHasAnEvaluationNote() throws Exception {
        Map<String, String> entries = new LegacyZipArtifactExtractor()
                .textEntries(download("reenjoiitaly___buttheresmorecarfwork.zip"));
        String backlog = new String(Files.readAllBytes(Paths.get("EVALUATION_BACKLOG.md")), "UTF-8");

        assertEquals(new HashSet<>(Arrays.asList(
                "ActualConflict.java",
                "PotentialConflict.java",
                "RouteSegmentSource.java",
                "AirspaceReservation.java",
                "DeconflictionContext.java",
                "DeconflictionEntityType.java",
                "DeconflictionTester.java",
                "DeconflictionUtils.java",
                "EntityFactory.java",
                "ManeuverAreaEvent.java",
                "OrbitEvent.java",
                "StationaryReservationEvent.java",
                "TimingTriangleEvent.java"
        )), entries.keySet());
        for (String fileName : entries.keySet()) {
            assertTrue(backlog.contains("`" + fileName + "`"), "Missing coverage note for " + fileName);
        }
    }

    @Test
    void classifiesAutodrawJavaAsReferenceOnlyDueToLegacyDependencies() throws Exception {
        LegacyJavaArtifactAnalyzer.Analysis dafif = new LegacyJavaArtifactAnalyzer()
                .analyze(download("autodrawFiles/CarfDafifManager.java"));
        LegacyJavaArtifactAnalyzer.Analysis mission = new LegacyJavaArtifactAnalyzer()
                .analyze(download("autodrawFiles/MissionManager.java"));

        assertFalse(dafif.isCopyCandidate());
        assertFalse(mission.isCopyCandidate());
        assertTrue(dafif.getExternalFamilies().contains("com.luciad"));
        assertTrue(dafif.getExternalFamilies().contains("gov.faa"));
        assertTrue(mission.getExternalFamilies().contains("com.luciad"));
        assertTrue(mission.getExternalFamilies().contains("gov.faa"));
    }

    @Test
    void inventoriesCleanCarfBackupAsPostgresReferenceDataDump() throws Exception {
        PostgresCustomDumpAnalyzer.Analysis analysis =
                new PostgresCustomDumpAnalyzer().analyze(download("CLEANCARF.backup"));

        assertTrue(analysis.isPostgresCustomDump());
        assertEquals("CarfdB2", analysis.getDatabaseName());
        assertFalse(analysis.isPgRestoreAvailable());
        assertTrue(analysis.getTables().contains("t_Separations"));
        assertTrue(analysis.getTables().contains("t_GroupSeparations"));
        assertTrue(analysis.getTables().contains("t_AirspaceMap"));
        assertTrue(analysis.getTables().contains("t_Navaids"));
        assertTrue(analysis.getCopyTables().contains("t_CarfMessage"));
        assertTrue(analysis.getRelevantTables().contains("t_Reservation"));
        assertTrue(analysis.getSequenceValues().stream().anyMatch(v -> v.startsWith("Mission_id_sequence=")));
        assertFalse(analysis.isRowExtractionAvailable());
        assertTrue(analysis.getRowExtractionDiagnostic().contains("pg_restore"));
    }

    @Test
    void classifiesLegacyCarfSpreadsheetsByExtractableContent() throws Exception {
        LegacySpreadsheetArtifactAnalyzer analyzer = new LegacySpreadsheetArtifactAnalyzer();

        LegacySpreadsheetArtifactAnalyzer.Analysis fax =
                analyzer.analyze(download("CARF Fax Number.xls"));
        LegacySpreadsheetArtifactAnalyzer.Analysis testing =
                analyzer.analyze(download("carftesting.xls"));

        assertTrue(fax.isCompoundDocument());
        assertTrue(fax.getSheetNames().contains("test1"));
        assertTrue(fax.getContactTokens().stream().anyMatch(v -> v.contains("ZNY TMU")));
        assertTrue(fax.getContactTokens().stream().anyMatch(v -> v.contains("VACAPES")));
        assertTrue(fax.isReferenceOnly());

        assertTrue(testing.isCompoundDocument());
        assertTrue(testing.getSheetNames().contains("Sheet1"));
        assertTrue(testing.getSheetNames().contains("Sheet2"));
        assertTrue(testing.getExecutableRows().isEmpty());
        assertTrue(testing.isReferenceOnly());
    }

    @Test
    void marksConflictDisplayRarAttachmentBlockedBecauseHeadersAreEncrypted() throws Exception {
        byte[] attachment = new LegacyEmailAttachmentExtractor()
                .attachment(download("conflict display.eml"), "display.rar")
                .orElseThrow(() -> new AssertionError("Missing display.rar attachment"));

        LegacyRarArtifactAnalyzer.Analysis analysis = new LegacyRarArtifactAnalyzer().analyze(attachment);

        assertTrue(analysis.isRar());
        assertTrue(analysis.isEncryptedHeaders());
    }

    @Test
    void extractsGlobalAccountsFromLegacySpreadsheet() throws Exception {
        GlobalAccountKeywordResolver resolver =
                GlobalAccountKeywordResolver.fromLegacySpreadsheet(download("GLOBALACCOUNTS.xls"));

        assertTrue(resolver.isGlobalAccount("GPS"));
        assertTrue(resolver.isGlobalAccount("FDC"));
        assertTrue(resolver.isGlobalAccount("CARF"));
        assertEquals("NAV", resolver.defaultKeyword("GPS"));
        assertEquals("AIRSPACE", resolver.defaultKeyword("CARF"));
    }

    @Test
    void extractsFirReferenceIdentifiersFromLegacySpreadsheet() throws Exception {
        FirReferenceCatalog catalog = FirReferenceCatalog.fromLegacySpreadsheet(download("FIRs.xls"));

        assertTrue(catalog.containsIdentifier("ZOA"));
        assertTrue(catalog.containsIdentifier("ZAN"));
        assertTrue(catalog.containsIdentifier("ZBW"));
        assertTrue(catalog.containsIdentifier("CZVR"));
        assertTrue(catalog.containsIdentifier("KJFK"));
        assertTrue(catalog.descriptions().stream().anyMatch(v -> v.contains("VANCOUVER FIR")));
        assertTrue(catalog.descriptions().stream().anyMatch(v -> v.contains("JOHN F KENNEDY INTL")));
    }

    @Test
    void inventoriesVisualEvidenceArtifactsAsReferenceOnly() throws Exception {
        VisualArtifactAnalyzer analyzer = new VisualArtifactAnalyzer();

        VisualArtifactAnalyzer.Analysis checkin = analyzer.analyze(download("carf-checkin.png"));
        VisualArtifactAnalyzer.Analysis ptrOriginal = analyzer.analyze(download("ptr-02-original-conflict.png"));
        VisualArtifactAnalyzer.Analysis ptrNow = analyzer.analyze(download("ptr-02-now-conflict.png"));
        VisualArtifactAnalyzer.Analysis mrkoci = analyzer.analyze(download("mrkoci/JRLAT-TEST.png"));

        assertEquals(674, checkin.getWidth());
        assertEquals(436, checkin.getHeight());
        assertEquals(1011, ptrOriginal.getWidth());
        assertEquals(573, ptrOriginal.getHeight());
        assertEquals(1013, ptrNow.getWidth());
        assertEquals(525, ptrNow.getHeight());
        assertEquals(1039, mrkoci.getWidth());
        assertEquals(683, mrkoci.getHeight());
        assertTrue(checkin.isReferenceOnly());
        assertTrue(ptrOriginal.isReferenceOnly());
        assertTrue(ptrNow.isReferenceOnly());
        assertTrue(mrkoci.isReferenceOnly());
    }

    @Test
    void detectsDuplicateForwardedShortConflictScreenshots() throws Exception {
        VisualArtifactAnalyzer analyzer = new VisualArtifactAnalyzer();

        VisualArtifactAnalyzer.Analysis first =
                analyzer.analyze(download("conflicttoshort/90-91conflict.jpg"));
        VisualArtifactAnalyzer.Analysis forwarded =
                analyzer.analyze(download("fwdconflicttoshort/90-91conflict.jpg"));

        assertEquals(1366, first.getWidth());
        assertEquals(768, first.getHeight());
        assertEquals(first.getSha256(), forwarded.getSha256());
        assertTrue(analyzer.sameImage(
                download("conflicttoshort/90-91conflict.jpg"),
                download("fwdconflicttoshort/90-91conflict.jpg")));
    }

    private GeoCoordinate point(double latitude, double longitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build();
    }

    private Path download(String relative) {
        return Paths.get(System.getProperty("user.home"), "Downloads", relative);
    }

    private Set<String> zipEntryNames(byte[] bytes) throws Exception {
        Set<String> names = new LinkedHashSet<>();
        try (ZipInputStream zip = new ZipInputStream(new ByteArrayInputStream(bytes))) {
            ZipEntry entry;
            while ((entry = zip.getNextEntry()) != null) {
                if (!entry.isDirectory()) {
                    names.add(entry.getName());
                }
            }
        }
        return names;
    }

    private byte[] zipEntry(byte[] bytes, String wantedName) throws Exception {
        try (ZipInputStream zip = new ZipInputStream(new ByteArrayInputStream(bytes))) {
            ZipEntry entry;
            while ((entry = zip.getNextEntry()) != null) {
                if (wantedName.equals(entry.getName())) {
                    java.io.ByteArrayOutputStream output = new java.io.ByteArrayOutputStream();
                    byte[] buffer = new byte[8192];
                    int read;
                    while ((read = zip.read(buffer)) >= 0) {
                        output.write(buffer, 0, read);
                    }
                    return output.toByteArray();
                }
            }
        }
        throw new AssertionError("Missing zip entry " + wantedName);
    }
}
