package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.DomesticNotamContractionDictionary;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamRecord;
import org.tash.extensions.notam.GlobalAccountKeywordResolver;

import java.nio.file.Path;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.nio.file.Files;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
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
