package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.DomesticNotamContractionDictionary;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamRecord;
import org.tash.extensions.notam.GlobalAccountKeywordResolver;
import org.tash.extensions.evaluation.LegacyArtifactTextExtractor;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.LinkedHashSet;
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
    void extractsAndParsesKeywordClarificationSpreadsheetRows() throws Exception {
        LegacyArtifactTextExtractor extractor = new LegacyArtifactTextExtractor();
        DomesticNotamParser parser = new DomesticNotamParser();

        assertSpreadsheetRowHasContractions(extractor, parser, "APCH 201101.xls",
                "APCH", "APCH=approach");
        assertSpreadsheetRowHasContractions(extractor, parser, "Apron Taxilane 201101.xls",
                "TAXILANE", "TAXILANE=taxilane", "LGTD=lighted");
        assertSpreadsheetRowHasContractions(extractor, parser, "Disabled 201101.xls",
                "DISABLED", "DISABLED=disabled", "ACFT=aircraft");
        assertSpreadsheetRowHasContractions(extractor, parser, "Dmsnt acft 201101.xls",
                "ACFT", "DMSTN=dismantled", "ACFT=aircraft");
        assertSpreadsheetRowHasContractions(extractor, parser, "Ft 201101.xls",
                "FT WIDE", "FT=feet");
        assertSpreadsheetRowHasContractions(extractor, parser, "Guard Lgts.xls",
                "GUARD", "GUARD=guard", "LGTS=lights");
        assertSpreadsheetRowHasContractions(extractor, parser, "Moored Ship 201101.xls",
                "MOORED", "MOORED=moored", "SHIP=ship");
        assertSpreadsheetRowHasContractions(extractor, parser, "RAMP PAEW 201101.xls",
                "PAEW", "RAMP=ramp", "PAEW=personnel and equipment working");
        assertSpreadsheetRowHasContractions(extractor, parser, "SFC 201101.xls",
                "SFC", "SFC=surface");
        assertSpreadsheetRowHasContractions(extractor, parser, "Ski Strip 201101.xls",
                "SKI", "SKI=ski", "STRIP=strip");
        assertSpreadsheetRowHasContractions(extractor, parser, "Snow 20110131.xls",
                "SNOW", "SNOW=snow");
        assertSpreadsheetRowHasContractions(extractor, parser, "Turnaround 201101.xls",
                "TURNAROUNDS", "TURNAROUNDS=turnaround");
        assertSpreadsheetRowHasContractions(extractor, parser, "Unavbl 201101.xls",
                "UNAVBL", "UNAVBL=unavailable");
        assertSpreadsheetRowHasContractions(extractor, parser, "Unmon 201101.xls",
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
    void loadsContractionsFromLegacyDom2Grammar() throws Exception {
        DomesticNotamContractionDictionary dictionary = DomesticNotamContractionDictionary.fromLegacyYcc(
                Paths.get(System.getProperty("user.home"), "Downloads", "Dom2.ycc"));
        DomesticNotamParser parser = new DomesticNotamParser(dictionary, new GlobalAccountKeywordResolver());

        assertTrue(dictionary.entries().size() > 250);
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
        DomesticNotamContractionDictionary dictionary = DomesticNotamContractionDictionary.fromLegacyYcc(
                Paths.get(System.getProperty("user.home"), "Downloads", "newDom2.ycc"));

        assertEquals("taxilane", dictionary.classify("TAXILANE").orElseThrow(AssertionError::new));
        assertEquals("taxilane", dictionary.classify("TAXILANES").orElseThrow(AssertionError::new));
        assertEquals("unavailable", dictionary.classify("UNABL").orElseThrow(AssertionError::new));
    }

    @Test
    void usesExtractedGlobalAccountResolver() throws Exception {
        GlobalAccountKeywordResolver resolver = GlobalAccountKeywordResolver.fromLegacySpreadsheet(
                Paths.get(System.getProperty("user.home"), "Downloads", "GLOBALACCOUNTS.xls"));
        DomesticNotamParseResult result = new DomesticNotamParser(resolver)
                .parseDetailed("!CARF ZZZZ CLOSED 1101011200-1101011300");

        assertTrue(result.isAccepted());
        assertEquals("AIRSPACE", result.getInferredKeyword());
        assertEquals("CLOSED", result.getRecord().getText());
    }

    @Test
    void parsesDomesticNotamsExtractedFromParserTesterWorkspace() throws Exception {
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

    private Set<String> parserTesterDomesticLines() throws Exception {
        java.nio.file.Path bundlePath = Paths.get(System.getProperty("user.home"), "Downloads", "Bundle.eml");
        byte[] bundle = new org.tash.extensions.evaluation.LegacyEmailAttachmentExtractor()
                .attachment(bundlePath, "davorJunk.zip")
                .orElseThrow(() -> new AssertionError("Missing davorJunk.zip"));
        byte[] daveWorkspace = zipEntry(bundle, "davorJunk/DaveWorkspace.zip");
        Set<String> lines = new LinkedHashSet<>();
        try (java.util.zip.ZipInputStream zip =
                     new java.util.zip.ZipInputStream(new java.io.ByteArrayInputStream(daveWorkspace))) {
            java.util.zip.ZipEntry entry;
            while ((entry = zip.getNextEntry()) != null) {
                if (!entry.getName().startsWith("DaveWorkspace/DomNotamParserTester/")
                        || !entry.getName().endsWith(".txt")) {
                    continue;
                }
                java.io.ByteArrayOutputStream output = new java.io.ByteArrayOutputStream();
                byte[] buffer = new byte[4096];
                int read;
                while ((read = zip.read(buffer)) >= 0) {
                    output.write(buffer, 0, read);
                }
                Arrays.stream(new String(output.toByteArray(), "ISO-8859-1").replace('\r', '\n').split("\\n"))
                        .map(String::trim)
                        .filter(line -> line.startsWith("!"))
                        .forEach(lines::add);
            }
        }
        return lines;
    }

    private void assertSpreadsheetRowHasContractions(LegacyArtifactTextExtractor extractor,
                                                     DomesticNotamParser parser,
                                                     String fileName,
                                                     String marker,
                                                     String... expectedContractions) throws Exception {
        String line = firstNotamContaining(extractor, keywordClarificationPath(fileName), marker);
        DomesticNotamParseResult result = parser.parseDetailed(line);

        assertTrue(result.isAccepted(), line + " should parse: " + result.getRejectionReason());
        for (String expected : expectedContractions) {
            assertTrue(result.getRecognizedContractions().contains(expected),
                    "Expected " + expected + " in " + result.getRecognizedContractions() + " from " + line);
        }
    }

    private String firstNotamContaining(LegacyArtifactTextExtractor extractor,
                                       Path path,
                                       String marker) throws Exception {
        return extractor.notamLines(path).stream()
                .filter(line -> line.contains(marker))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing " + marker + " row in " + path));
    }

    private Path keywordClarificationPath(String fileName) {
        return Paths.get(System.getProperty("user.home"), "Downloads",
                "fwdfwdkeywordsthatneedclarification", fileName);
    }

    private String domestic(String body) {
        return "!DCA LDN " + body + " 1012211200-1012211300";
    }

    private byte[] zipEntry(byte[] bytes, String wantedName) throws Exception {
        try (java.util.zip.ZipInputStream zip =
                     new java.util.zip.ZipInputStream(new java.io.ByteArrayInputStream(bytes))) {
            java.util.zip.ZipEntry entry;
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
