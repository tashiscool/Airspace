package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;

import static org.junit.jupiter.api.Assertions.*;

class UsnsIngestServiceTest {
    @Test
    void routesRawEnvelopeToDomesticParser() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope(
                "!ABC ABC RWY 04 CLSD 1001011200-1001011300"));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertEquals(1, result.getDomesticResults().size());
        assertTrue(result.getDomesticResults().get(0).isAccepted());
        assertEquals("RWY", result.getDomesticResults().get(0).getRecord().getKeyword());
    }

    @Test
    void routesCarfLikeBodyToCarfParser() {
        String carf = "A. TEST01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100\n"
                + "F. ETD 021300 MAR 2010 AVANA 021400\n"
                + "G. TAS: 480 KTAS";

        UsnsIngestResult result = new UsnsIngestService().parse(envelope(carf));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertEquals(1, result.getCarfMessages().size());
        assertEquals("TEST01", result.getCarfMessages().get(0).getActivityName());
    }

    @Test
    void preservesUnsupportedClassifiedTransactions() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("SNOWTAM 0123"));

        assertTrue(result.getErrors().isEmpty());
        assertEquals(1, result.getUnsupportedTransactions().size());
        assertEquals(UsnsTransactionType.SNOWTAM, result.getUnsupportedTransactions().get(0).getType());
    }

    @Test
    void routesIcaoAndCanadianNotamsToTypedFieldParser() {
        String body = "(A0001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)\n"
                + "(A0002/26 NOTAMC A) KZNY E) CANCEL TEST)\n"
                + "(1234/26 NOTAMJ A) CYUL E) CANADIAN TEST TIL APRX 2601011200)";

        UsnsIngestResult result = new UsnsIngestService().parse(envelope(body));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertEquals(3, result.getNotamFieldResults().size());
        assertTrue(result.getNotamFieldResults().stream().anyMatch(fields ->
                "NOTAMN".equals(fields.getNotamType())
                        && "QRTCA".equals(fields.getQCode())
                        && fields.isHasGeometry()));
        assertTrue(result.getNotamFieldResults().stream().anyMatch(fields ->
                "NOTAMC".equals(fields.getNotamType())
                        && !fields.isHasGeometry()
                        && fields.getDiagnostics().stream().anyMatch(d -> d.contains("No compact coordinate"))));
        assertTrue(result.getNotamFieldResults().stream().anyMatch(fields ->
                "NOTAMJ".equals(fields.getNotamType())
                        && "CYUL".equals(fields.getAField())));
    }

    @Test
    void propagatesMalformedServiceCommandDiagnostics() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("(SVC RQ)\n(SVC TBL DOM)"));

        assertFalse(result.getErrors().isEmpty());
        assertTrue(result.getTransactionIngestResults().stream()
                .anyMatch(transaction -> transaction.getServiceRequest() != null
                        && !transaction.getServiceRequest().isAccepted()
                        && transaction.getErrors().stream().anyMatch(error -> error.contains("domain is missing"))));
        assertTrue(result.getTransactionIngestResults().stream()
                .anyMatch(transaction -> transaction.getServiceTable() != null
                        && !transaction.getServiceTable().isAccepted()
                        && transaction.getErrors().stream().anyMatch(error -> error.contains("operation is missing"))));
    }

    @Test
    void rejectsMalformedFdcAcknowledgementBeforeExecution() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("R FDC 0/1234 12"));

        assertFalse(result.getTransactionResults().isAccepted());
        assertFalse(result.getErrors().isEmpty());
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }
}
