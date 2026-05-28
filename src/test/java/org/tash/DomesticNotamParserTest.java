package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.DomesticNotamContractionDictionary;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamRecord;
import org.tash.extensions.notam.GlobalAccountKeywordResolver;
import org.tash.extensions.notam.LowVisibilityProcedureAssessment;
import org.tash.extensions.notam.LowVisibilityProcedureAssessmentService;
import org.tash.extensions.notam.LowVisibilityProcedureProfile;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;

import java.nio.file.Path;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.nio.file.Files;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

class DomesticNotamParserTest {
    @Test
    void parsesDom1VisualParserFixture() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!DCA LDN NAV VOR OTS 12 0708051600-0708052359EST");

        assertEquals(DomesticNotamRecord.Type.NEW, record.getType());
        assertEquals("DCA", record.getAccountability());
        assertEquals("LDN", record.getLocation());
        assertEquals("NAV", record.getKeyword());
        assertEquals("VOR OTS 12", record.getText());
        assertEquals(ZonedDateTime.of(2007, 8, 5, 16, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2007, 8, 5, 23, 59, 0, 0, ZoneOffset.UTC), record.getEffectiveEnd());
        assertTrue(record.isEstimatedEnd());
    }

    @Test
    void parsesEditAndComment() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!DCA 12/345 LDN AIRSPACE CLOSED WEF 1010151200 TIL 1010151800 ^^ reviewed");

        assertEquals(DomesticNotamRecord.Type.EDIT, record.getType());
        assertEquals("12/345", record.getNotamNumber());
        assertEquals("AIRSPACE", record.getKeyword());
        assertEquals("CLOSED", record.getText());
        assertEquals("reviewed", record.getComment());
        assertEquals(ZonedDateTime.of(2010, 10, 15, 12, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2010, 10, 15, 18, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveEnd());
    }

    @Test
    void parsesCancellation() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!DCA C12/345 cancellation text");

        assertEquals(DomesticNotamRecord.Type.CANCEL, record.getType());
        assertEquals("C12/345", record.getCancelNumber());
        assertEquals("cancellation text", record.getText());
    }

    @Test
    void parsesExpandedDomGrammarKeywordsAndTwelveDigitDateRanges() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!DCA LDN IAP PROCEDURE AMDT 101221120030 - 101221130045EST");

        assertEquals("IAP", record.getKeyword());
        assertEquals("PROCEDURE AMDT", record.getText());
        assertEquals(ZonedDateTime.of(2010, 12, 21, 12, 0, 30, 0, ZoneOffset.UTC), record.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2010, 12, 21, 13, 0, 45, 0, ZoneOffset.UTC), record.getEffectiveEnd());
        assertTrue(record.isEstimatedEnd());
    }

    @Test
    void parsesPermDurationFromExpandedDomGrammar() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!DCA LDN ROUTE CHANGED 1012211200-PERM");

        assertEquals("ROUTE", record.getKeyword());
        assertEquals("CHANGED", record.getText());
        assertEquals(ZonedDateTime.of(2010, 12, 21, 12, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveStart());
        assertEquals(record.getEffectiveStart().plusYears(100), record.getEffectiveEnd());
    }

    @Test
    void allowsCanadianCrossoverEditWithoutKeyword() {
        DomesticNotamRecord record = new DomesticNotamParser()
                .parse("!CYHZ 12/345 CYHZ RUNWAY CLOSED 1012211200-1012211300");

        assertEquals(DomesticNotamRecord.Type.EDIT, record.getType());
        assertNull(record.getKeyword());
        assertEquals("RUNWAY CLOSED", record.getText());
        assertEquals(ZonedDateTime.of(2010, 12, 21, 12, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2010, 12, 21, 13, 0, 0, 0, ZoneOffset.UTC), record.getEffectiveEnd());
    }

    @Test
    void returnsStructuredRejectionAndGlobalAccountKeywordInference() {
        DomesticNotamParser parser = new DomesticNotamParser();
        DomesticNotamParseResult inferred = parser.parseDetailed(
                "!GPS LDN VOR OTS 1012211200-1012211300");
        DomesticNotamParseResult rejected = parser.parseDetailed("DCA LDN NAV VOR OTS");

        assertTrue(inferred.isAccepted());
        assertEquals("NAV", inferred.getInferredKeyword());
        assertEquals("VOR OTS", inferred.getRecord().getText());
        assertFalse(inferred.getWarnings().isEmpty());
        assertFalse(rejected.isAccepted());
        assertTrue(rejected.getRejectionReason().contains("must start"));
    }

    @Test
    void classifiesDom2KeywordClarificationContractions() {
        DomesticNotamParseResult result = new DomesticNotamParser().parseDetailed(
                "!DCA LDN APRON TAXILANE UNABL APCH LGTS SFC 1012211200-1012211300");

        assertTrue(result.isAccepted());
        assertTrue(result.getRecognizedContractions().contains("TAXILANE=taxilane"));
        assertTrue(result.getRecognizedContractions().contains("UNABL=unavailable"));
        assertTrue(result.getRecognizedContractions().contains("APCH=approach"));
        assertTrue(result.getRecognizedContractions().contains("LGTS=lights"));
        assertTrue(result.getRecognizedContractions().contains("SFC=surface"));
    }

    @Test
    void parsesRepresentativeKeywordClarificationRows() {
        DomesticNotamParser parser = new DomesticNotamParser();

        assertSpreadsheetRowHasContractions(parser,
                "APCH", "APCH=approach");
        assertSpreadsheetRowHasContractions(parser,
                "TAXILANE", "TAXILANE=taxilane", "LGTD=lighted");
        assertSpreadsheetRowHasContractions(parser,
                "DISABLED", "DISABLED=disabled", "ACFT=aircraft");
        assertSpreadsheetRowHasContractions(parser,
                "ACFT", "DMSTN=dismantled", "ACFT=aircraft");
        assertSpreadsheetRowHasContractions(parser,
                "FT WIDE", "FT=feet");
        assertSpreadsheetRowHasContractions(parser,
                "GUARD", "GUARD=guard", "LGTS=lights");
        assertSpreadsheetRowHasContractions(parser,
                "MOORED", "MOORED=moored", "SHIP=ship");
        assertSpreadsheetRowHasContractions(parser,
                "PAEW", "PAEW=personnel and equipment working");
        assertSpreadsheetRowHasContractions(parser,
                "SFC", "SFC=surface");
        assertSpreadsheetRowHasContractions(parser,
                "SKI", "SKI=ski", "STRIP=strip");
        assertSpreadsheetRowHasContractions(parser,
                "SNOW", "SNOW=snow");
        assertSpreadsheetRowHasContractions(parser,
                "TURNAROUNDS", "TURNAROUNDS=turnaround");
        assertSpreadsheetRowHasContractions(parser,
                "UNAVBL", "UNAVBL=unavailable");
        assertSpreadsheetRowHasContractions(parser,
                "UNMON", "UNMON=unmonitored");
    }

    @Test
    void classifiesDom2ReducerQCodesForConcreteTestCases() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult vor = parser.parseDetailed(domestic("NAV VOR CHECKPOINT"));
        DomesticNotamParseResult vortac = parser.parseDetailed(domestic("NAV VORTAC UNMON"));
        DomesticNotamParseResult guardLights = parser.parseDetailed(domestic("TWY E RWY GUARD LGTS OTS"));
        DomesticNotamParseResult arff = parser.parseDetailed(domestic("SVC ARFF UNAVBL"));
        DomesticNotamParseResult pje = parser.parseDetailed(domestic("AIRSPACE PJE 5 NMR ABC 110/020"));
        DomesticNotamParseResult runwayPcl = parser.parseDetailed(domestic("RWY 10/28 LIGHTS PCL OTS"));

        assertEquals("NV", vor.getQ23());
        assertEquals("NT", vortac.getQ23());
        assertEquals("LS", guardLights.getQ23());
        assertEquals("FF", arff.getQ23());
        assertEquals("WP", pje.getQ23());
        assertEquals("LE", runwayPcl.getQ23());
        assertEquals("XX", runwayPcl.getQ45());
        assertTrue(runwayPcl.getQCodeReason().contains("pilot-controlled"));
    }

    @Test
    void classifiesExpandedDom2SemanticFamiliesAndReducers() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult runwaySnow = parser.parseDetailed(domestic("RWY 10 ICE SLUSH WINDROWS"));
        DomesticNotamParseResult taxiwayClosed = parser.parseDetailed(domestic("TWY A CLSD"));
        DomesticNotamParseResult aerobatic = parser.parseDetailed(domestic("AIRSPACE AEROBATIC AREA 5 NMR ABC"));
        DomesticNotamParseResult restricted = parser.parseDetailed(domestic("AIRSPACE R1234 ACTIVE"));
        DomesticNotamParseResult tdwr = parser.parseDetailed(domestic("SVC TDWR OTS"));
        DomesticNotamParseResult llwas = parser.parseDetailed(domestic("SVC LLWAS UNAVBL"));
        DomesticNotamParseResult obstruction = parser.parseDetailed(domestic("OBST CRANE LGTD"));

        assertEquals("RWY", runwaySnow.getSemanticFacilityFamily());
        assertEquals("ICE", runwaySnow.getSemanticCondition());
        assertEquals("DOM2.SURFACE.ICE", runwaySnow.getReducerRuleId());
        assertEquals("TWY", taxiwayClosed.getSemanticFacilityFamily());
        assertEquals("CLOSED", taxiwayClosed.getSemanticAction());
        assertEquals("AIRSPACE", aerobatic.getSemanticFacilityFamily());
        assertEquals("AEROBATIC", aerobatic.getSemanticCondition());
        assertEquals("RESTRICTED_ZONE", restricted.getSemanticCondition());
        assertEquals("TDWR", tdwr.getSemanticCondition());
        assertEquals("LLWAS", llwas.getSemanticCondition());
        assertEquals("OBST", obstruction.getSemanticFacilityFamily());
        assertNotNull(runwaySnow.getSemanticClassification());
        assertTrue(runwaySnow.getReducerName().contains("snow/surface"));
    }

    @Test
    void classifiesRvrAndLowVisibilityProcedureNotamsAsCoordinationRelevant() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult rvr = parser.parseDetailed(domestic("RWY 04L RVRT U/S"));
        DomesticNotamParseResult rvrMid = parser.parseDetailed(domestic("RWY 04L RVRM U/S"));
        DomesticNotamParseResult rvrRollout = parser.parseDetailed(domestic("RWY 04L RVRR U/S"));
        DomesticNotamParseResult rvrAll = parser.parseDetailed(domestic("AD AP RVR ALL U/S"));
        DomesticNotamParseResult legacySvcRvr = parser.parseDetailed(domestic("SVC RWY 04L RVRT OTS"));
        DomesticNotamParseResult compatibilitySvcRvr = parser.parseDetailed(domestic("SVC RVR ALL OTS"));
        DomesticNotamParseResult lowVisibility = parser.parseDetailed(domestic("SVC SMGCS LOW VISIBILITY PROC IN USE"));

        assertTrue(rvr.isAccepted(), rvr.getRejectionReason());
        assertEquals("DOM2.RWY.RVR", rvr.getReducerRuleId());
        assertEquals("RWY", rvr.getSemanticFacilityFamily());
        assertEquals("RVRT", rvr.getSemanticCondition());
        assertEquals("UNAVAILABLE", rvr.getSemanticAction());
        assertEquals("FT", rvr.getQ23());
        assertTrue(rvr.getRecognizedContractions().contains("RVRT=runway visual range touchdown"));
        assertTrue(rvr.getWarnings().stream().anyMatch(warning -> warning.contains("low-visibility departure")));
        assertEquals("RVRM", rvrMid.getSemanticCondition());
        assertEquals("RVRR", rvrRollout.getSemanticCondition());
        assertEquals("DOM2.AD.RVR_ALL", rvrAll.getReducerRuleId());
        assertEquals("AD", rvrAll.getSemanticFacilityFamily());
        assertEquals("RVR_ALL", rvrAll.getSemanticCondition());
        assertEquals("DOM2.RWY.RVR", legacySvcRvr.getReducerRuleId());
        assertEquals("RWY", legacySvcRvr.getSemanticFacilityFamily());
        assertEquals("DOM2.SVC.RVR", compatibilitySvcRvr.getReducerRuleId());
        assertEquals("SVC", compatibilitySvcRvr.getSemanticFacilityFamily());

        assertTrue(lowVisibility.isAccepted(), lowVisibility.getRejectionReason());
        assertEquals("DOM2.SVC.LOW_VISIBILITY_PROCEDURE", lowVisibility.getReducerRuleId());
        assertEquals("LOW_VISIBILITY_PROCEDURE", lowVisibility.getSemanticCondition());
        assertTrue(lowVisibility.getRecognizedContractions().contains("SMGCS=surface movement guidance and control system"));
        assertTrue(lowVisibility.getWarnings().stream().anyMatch(warning -> warning.contains("FAA/ICAO")));
    }

    @Test
    void rejectsLegacyDom2MalformedServiceRvrWithoutRunwayKeyword() {
        DomesticNotamParseResult result = new DomesticNotamParser().parseDetailed(
                "!DCA LDN SVC 12/30 RVR OTS 1012211200-1012211300");

        assertFalse(result.isAccepted());
        assertTrue(result.getRejectionReason().contains("RWY <id> RVR"));
    }

    @Test
    void rejectsLegacyDom2MalformedBareRunwayObjectRows() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult runwayNowBareId = parser.parseDetailed(
                "!DCA LDN RWY NOW 12/30 1012211200-1012211300");
        DomesticNotamParseResult runwayWarningBareId = parser.parseDetailed(
                "!DCA LDN RWY WARNING EAST OF 12/30/TWY A 1012211200-1012211300");

        assertFalse(runwayNowBareId.isAccepted());
        assertTrue(runwayNowBareId.getRejectionReason().contains("RWY <id>"));
        assertFalse(runwayWarningBareId.isAccepted());
        assertTrue(runwayWarningBareId.getRejectionReason().contains("runway/taxiway object"));
    }

    @Test
    void rejectsLegacyDom2MalformedNavRunwayLandingAidRows() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult malformedIls = parser.parseDetailed(
                "!DCA LDN NAV RWY ILS OTS 1012211200-1012211300");
        DomesticNotamParseResult malformedCat = parser.parseDetailed(
                "!DCA LDN NAV RWY ILS CAT II NA 1012211200-1012211300");

        assertFalse(malformedIls.isAccepted());
        assertTrue(malformedIls.getRejectionReason().contains("landing aid"));
        assertFalse(malformedCat.isAccepted());
        assertTrue(malformedCat.getRejectionReason().contains("landing aid"));
    }

    @Test
    void classifiesApproachMinimaLightingAndSurfaceFrictionAsOperationalSafetyConstraints() {
        DomesticNotamParser parser = new DomesticNotamParser();

        DomesticNotamParseResult ilsCat = parser.parseDetailed(domestic("NAV ILS RWY 04L CAT II NA"));
        DomesticNotamParseResult papi = parser.parseDetailed(domestic("RWY 04L PAPI OTS"));
        DomesticNotamParseResult braking = parser.parseDetailed(domestic("RWY 04L BA POOR MU 20"));

        assertTrue(ilsCat.isAccepted(), ilsCat.getRejectionReason());
        assertEquals("DOM2.NAV.APPROACH_MINIMA", ilsCat.getReducerRuleId());
        assertEquals("APPROACH_MINIMA", ilsCat.getSemanticCondition());
        assertTrue(ilsCat.getWarnings().stream().anyMatch(warning -> warning.contains("approach capability")));

        assertTrue(papi.isAccepted(), papi.getRejectionReason());
        assertEquals("DOM2.LIGHTING.APPROACH", papi.getReducerRuleId());
        assertEquals("APPROACH_LIGHTING", papi.getSemanticCondition());
        assertTrue(papi.getWarnings().stream().anyMatch(warning -> warning.contains("approach/landing minima")));

        assertTrue(braking.isAccepted(), braking.getRejectionReason());
        assertEquals("DOM2.SURFACE.FRICTION", braking.getReducerRuleId());
        assertEquals("FRICTION", braking.getSemanticCondition());
        assertTrue(braking.getRecognizedContractions().contains("BA=braking action"));
        assertTrue(braking.getRecognizedContractions().contains("MU=friction coefficient"));
    }

    @Test
    void classifiesExpandedDomesticKeywordFamiliesAsTypedConstraints() {
        DomesticNotamParser parser = new DomesticNotamParser();
        Map<String, String> examples = new LinkedHashMap<>();
        examples.put("AD", "AD AP CLSD");
        examples.put("COM", "COM REMOTE COM OUTLET OTS");
        examples.put("IAP", "IAP RNAV RWY 04L NA");
        examples.put("ROUTE", "ROUTE J60 CHANGED");
        examples.put("SPECIAL", "SPECIAL MILITARY ACTIVITY");
        examples.put("SECURITY", "SECURITY TFR IN EFFECT");
        examples.put("ODP", "ODP RWY 04L AMDT");
        examples.put("SID", "SID BETTE THREE NA");
        examples.put("STAR", "STAR CAMRN FOUR NA");
        examples.put("CHART", "CHART IAP PAGE UPDATED");
        examples.put("DATA", "DATA DIGITAL OBSTACLE FILE UPDATED");

        Map<String, String> expectedRulePrefix = new LinkedHashMap<>();
        expectedRulePrefix.put("AD", "DOM2.AD.");
        expectedRulePrefix.put("COM", "DOM2.COM.");
        expectedRulePrefix.put("IAP", "DOM2.IAP.");
        expectedRulePrefix.put("ROUTE", "DOM2.ROUTE.");
        expectedRulePrefix.put("SPECIAL", "DOM2.SPECIAL.");
        expectedRulePrefix.put("SECURITY", "DOM2.SECURITY.");
        expectedRulePrefix.put("ODP", "DOM2.ODP.");
        expectedRulePrefix.put("SID", "DOM2.SID.");
        expectedRulePrefix.put("STAR", "DOM2.STAR.");
        expectedRulePrefix.put("CHART", "DOM2.CHART.");
        expectedRulePrefix.put("DATA", "DOM2.DATA.");

        for (Map.Entry<String, String> entry : examples.entrySet()) {
            DomesticNotamParseResult result = parser.parseDetailed(domestic(entry.getValue()));
            assertTrue(result.isAccepted(), entry.getKey() + ": " + result.getRejectionReason());
            assertTrue(result.getReducerRuleId().startsWith(expectedRulePrefix.get(entry.getKey())),
                    entry.getKey() + " reduced to " + result.getReducerRuleId());
            assertNotEquals("DOM2.UNMATCHED", result.getReducerRuleId());
        }
    }

    @Test
    void lowVisibilityAssessmentKeepsReportedRvrEquipmentAndProcedureTerminologySeparate() {
        WeatherProductParseResult speci = new WeatherProductParser().parse(
                "SPECI KJFK 281655Z 04005KT 1/8SM R04L/1000FT FG VV002 08/08 A2992 RMK AO2",
                null);
        DomesticNotamParseResult rvrEquipment = new DomesticNotamParser().parseDetailed(
                "!DCA 05/777 JFK RWY 04L RVRT U/S 2605281200-2605281800");
        DomesticNotamParseResult procedureText = new DomesticNotamParser().parseDetailed(
                "!DCA 05/778 JFK SVC SMGCS LOW VISIBILITY PROC IN USE 2605281200-2605281800");
        LowVisibilityProcedureProfile profile = LowVisibilityProcedureProfile.builder()
                .airportId("KJFK")
                .localProcedureName("SMGCS / airport low-visibility equivalent")
                .faaTerminology("SMGCS")
                .icaoTerminology("LVO/LVP")
                .advisoryRvrThresholdFeet(1200)
                .smgcsAvailable(true)
                .lvpEquivalentAvailable(true)
                .source("local-fixture")
                .sourceVersion("2026-05")
                .rvrComponents(List.of("TDZ", "MID", "ROLLOUT"))
                .reviewAuthorities(List.of("ATIS", "TOWER_AIRPORT_OPS", "COMPANY_MINIMA", "LOCAL_AIRPORT_PROCEDURES"))
                .build();

        LowVisibilityProcedureAssessment assessment = new LowVisibilityProcedureAssessmentService()
                .assess("KJFK", profile, List.of(speci), List.of(rvrEquipment, procedureText));

        assertEquals(1000.0, assessment.getReportedRvrFeet());
        assertEquals("RVRT UNAVAILABLE", assessment.getRvrEquipmentStatus());
        assertTrue(assessment.getLowVisibilityProcedureTerminology().contains("SMGCS"));
        assertEquals("DELAY", assessment.getRecommendedAction());
        assertEquals("CONFIRM PROCEDURE STATE", assessment.getActionSublabel());
        assertTrue(assessment.isSeparateArtifactsRetained());
        assertTrue(assessment.getSourceRefs().stream().anyMatch(ref -> ref.startsWith("WEATHER:")));
        assertTrue(assessment.getSourceRefs().stream().anyMatch(ref -> ref.startsWith("NOTAM:")));
        assertTrue(assessment.getReviewMessages().stream().anyMatch(message -> message.contains("ATIS")));
        assertTrue(assessment.getReviewMessages().stream().anyMatch(message -> message.contains("Tower/airport ops")));
        assertTrue(assessment.getDiagnostics().stream().anyMatch(message -> message.contains("separate artifacts")));
    }

    @Test
    void rejectsAirportSnowConditionWithoutRunwayContext() {
        DomesticNotamParseResult result = new DomesticNotamParser().parseDetailed(
                "!DCA LDN AD SNOW ICE SLUSH 1012211200-1012211300");

        assertFalse(result.isAccepted());
        assertTrue(result.getRejectionReason().contains("Snow/surface condition"));
    }

    @Test
    void semanticFallbackKeepsAcceptedRecordWithDiagnosticWarning() {
        DomesticNotamParseResult result = new DomesticNotamParser().parseDetailed(
                domestic("RAMP UNUSUAL LEGACY TEXT"));

        assertTrue(result.isAccepted());
        assertEquals("DOM2.UNMATCHED", result.getReducerRuleId());
        assertTrue(result.getWarnings().stream().anyMatch(warning -> warning.contains("No DOM2 semantic reducer")));
    }

    @Test
    void loadsContractionsFromLegacyDom2Grammar() throws Exception {
        Path ycc = Files.createTempFile("dom2-contractions", ".ycc");
        Files.write(ycc, java.util.Arrays.asList(
                "'AZM' azm;",
                "'UNMNT' unmonitored;",
                "'UNABL' unavailable;"));
        DomesticNotamContractionDictionary dictionary = DomesticNotamContractionDictionary.fromLegacyYcc(ycc);
        DomesticNotamParser parser = new DomesticNotamParser(dictionary, new GlobalAccountKeywordResolver());

        assertEquals("azm", dictionary.classify("AZM").orElseThrow(AssertionError::new));
        assertEquals("unmonitored", dictionary.classify("UNMNT").orElseThrow(AssertionError::new));
        assertEquals("unavailable", dictionary.classify("UNABL").orElseThrow(AssertionError::new));

        DomesticNotamParseResult result = parser.parseDetailed(
                "!DCA LDN NAV TACAN AZM UNMNT 1012211200-1012211300");

        assertTrue(result.isAccepted());
        assertEquals("NN", result.getQ23());
        assertTrue(result.getRecognizedContractions().contains("AZM=azm"));
        assertTrue(result.getRecognizedContractions().contains("UNMNT=unmonitored"));
    }

    @Test
    void dom2DiffAddedContractionsArePresentInGrammarLoadedDictionary() throws Exception {
        Path ycc = Files.createTempFile("new-dom2-contractions", ".ycc");
        Files.write(ycc, java.util.Arrays.asList(
                "'TAXILANE' taxilane;",
                "'TAXILANES' taxilane;",
                "'UNABL' unavailable;"));
        DomesticNotamContractionDictionary dictionary = DomesticNotamContractionDictionary.fromLegacyYcc(ycc);

        assertEquals("taxilane", dictionary.classify("TAXILANE").orElseThrow(AssertionError::new));
        assertEquals("taxilane", dictionary.classify("TAXILANES").orElseThrow(AssertionError::new));
        assertEquals("unavailable", dictionary.classify("UNABL").orElseThrow(AssertionError::new));
    }

    @Test
    void usesGlobalAccountResolver() {
        GlobalAccountKeywordResolver resolver = new GlobalAccountKeywordResolver();
        DomesticNotamParseResult result = new DomesticNotamParser(resolver)
                .parseDetailed("!CARF ZZZZ CLOSED 1101011200-1101011300");

        assertTrue(result.isAccepted());
        assertEquals("AIRSPACE", result.getInferredKeyword());
        assertEquals("CLOSED", result.getRecord().getText());
    }

    @Test
    void parsesDomesticNotamsFromParserTesterExamples() {
        Set<String> domesticLines = parserTesterDomesticLines();
        DomesticNotamParser parser = new DomesticNotamParser();

        assertTrue(domesticLines.contains("!DCA LDN NAV VOR OTS WEF 0708051600-0708052359"));
        assertTrue(domesticLines.contains("!DCA LDN NAV VOR OTS WEF 07A08051600-0708052359"));

        DomesticNotamRecord valid = parser.parse("!DCA LDN NAV VOR OTS WEF 0708051600-0708052359");
        DomesticNotamParseResult malformed = parser.parseDetailed("!DCA LDN NAV VOR OTS WEF 07A08051600-0708052359");

        assertEquals("DCA", valid.getAccountability());
        assertEquals("LDN", valid.getLocation());
        assertEquals("NAV", valid.getKeyword());
        assertEquals("VOR OTS", valid.getText());
        assertEquals(ZonedDateTime.of(2007, 8, 5, 16, 0, 0, 0, ZoneOffset.UTC), valid.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2007, 8, 5, 23, 59, 0, 0, ZoneOffset.UTC), valid.getEffectiveEnd());
        assertFalse(malformed.isAccepted());
        assertTrue(malformed.getRejectionReason().contains("Invalid WEF date"));
    }

    private Set<String> parserTesterDomesticLines() {
        Set<String> lines = new LinkedHashSet<>();
        lines.add("!DCA LDN NAV VOR OTS WEF 0708051600-0708052359");
        lines.add("!DCA LDN NAV VOR OTS WEF 07A08051600-0708052359");
        return lines;
    }

    private void assertSpreadsheetRowHasContractions(DomesticNotamParser parser,
                                                     String marker,
                                                     String... expectedContractions) {
        String line = keywordClarificationRows().get(marker);
        DomesticNotamParseResult result = parser.parseDetailed(line);

        assertTrue(result.isAccepted(), line + " should parse: " + result.getRejectionReason());
        for (String expected : expectedContractions) {
            assertTrue(result.getRecognizedContractions().contains(expected),
                    "Expected " + expected + " in " + result.getRecognizedContractions() + " from " + line);
        }
    }

    private Map<String, String> keywordClarificationRows() {
        Map<String, String> rows = new LinkedHashMap<>();
        rows.put("APCH", domestic("RWY 10 APCH OTS"));
        rows.put("TAXILANE", domestic("APRON TAXILANE LGTD"));
        rows.put("DISABLED", domestic("RAMP DISABLED ACFT"));
        rows.put("ACFT", domestic("RAMP DMSTN ACFT"));
        rows.put("FT WIDE", domestic("TWY A 50 FT WIDE"));
        rows.put("GUARD", domestic("TWY A GUARD LGTS OTS"));
        rows.put("MOORED", domestic("RAMP MOORED SHIP"));
        rows.put("PAEW", domestic("RAMP PAEW"));
        rows.put("SFC", domestic("RWY 10 SFC WIP"));
        rows.put("SKI", domestic("RWY SKI STRIP CLSD"));
        rows.put("SNOW", domestic("RWY 10 SNOW"));
        rows.put("TURNAROUNDS", domestic("RWY 10 TURNAROUNDS CLSD"));
        rows.put("UNAVBL", domestic("NAV VOR UNAVBL"));
        rows.put("UNMON", domestic("NAV VOR UNMON"));
        return rows;
    }

    private String domestic(String body) {
        return "!DCA LDN " + body + " 1012211200-1012211300";
    }

}
