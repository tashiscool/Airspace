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
