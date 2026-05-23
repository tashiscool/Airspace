package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.messaging.UsnsMessageEnvelope;
import org.tash.extensions.messaging.transaction.FdcAcknowledgementCommand;
import org.tash.extensions.messaging.transaction.ServiceRequestCommand;
import org.tash.extensions.messaging.transaction.ServiceTableCommand;
import org.tash.extensions.messaging.transaction.UsnsTransactionParseResult;
import org.tash.extensions.messaging.transaction.UsnsTransactionSplitter;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;

import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class UsnsTransactionSplitterTest {
    @Test
    void splitsAndClassifiesLegacyTransactionFamilies() {
        UsnsMessageEnvelope envelope = UsnsMessageEnvelope.builder()
                .originAddress("CYUL")
                .body("!ABC ABC RWY 04 CLSD\n"
                        + "!FDC 0/1234 ZNY AIRBORNE TO GROUND LASER ACTIVITY\n"
                        + "(A1234/10 NOTAMN A) KJFK B) 1001011200 C) 1001011300 E) TEST)\n"
                        + "(A1235/10 NOTAMR A) KJFK B) 1001011200 C) 1001011300 E) TEST)\n"
                        + "(A1236/10 NOTAMC A) KJFK B) 1001011200 C) 1001011300 E) TEST)\n"
                        + "(1234/10 NOTAMJ A) CYUL B) 1001011200 C) 1001011300 E) TEST)\n"
                        + "SNOWTAM 0123\n"
                        + "BIRDTAM 0123\n"
                        + "ASHTAM 0123\n"
                        + "GENOT RWA 0123\n"
                        + ")SVC RQ DOM LOC=PDT\n"
                        + ")SVC TBL DOM LST CROSSOVER\n"
                        + "RGR FDC 0/1234 AB")
                .build();

        UsnsTransactionParseResult result = new UsnsTransactionSplitter().split(envelope);

        assertTrue(result.isAccepted(), result.getErrors().toString());
        assertTrue(result.getTransactions().stream().map(t -> t.getType()).collect(Collectors.toList())
                .contains(UsnsTransactionType.DOMESTIC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.FDC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ICAO_NOTAMN));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ICAO_NOTAMR));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ICAO_NOTAMC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.CANADIAN_DOMESTIC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SNOWTAM));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.BIRDTAM));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ASHTAM));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.GENOT));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SERVICE_REQUEST));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SERVICE_TABLE));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.FDC_ACK));
    }

    @Test
    void rejectsBatchWhenMarkedTransactionCannotBeClassified() {
        UsnsMessageEnvelope envelope = UsnsMessageEnvelope.builder()
                .body("R FDC 0/1234 12")
                .build();

        UsnsTransactionParseResult result = new UsnsTransactionSplitter().split(envelope);

        assertFalse(result.isAccepted());
        assertEquals(UsnsTransactionType.UNKNOWN, result.getTransactions().get(0).getType());
    }

    @Test
    void parsesCommandSummaries() {
        ServiceRequestCommand request = ServiceRequestCommand.parse(")SVC RQ DOM LOC=PDT");
        ServiceTableCommand table = ServiceTableCommand.parse(")SVC TBL DOM LST CROSSOVER");
        FdcAcknowledgementCommand ack = FdcAcknowledgementCommand.parse("RGR FDC 0/1234 AB");

        assertEquals("RQ", request.getService());
        assertEquals("DOM", request.getDomain());
        assertEquals("LOC=PDT", request.getOperation());
        assertEquals("TBL", table.getService());
        assertEquals("LST", table.getOperation());
        assertTrue(ack.isAccepted());
        assertEquals("AB", ack.getInitials());
    }

    @Test
    void serviceCommandsRejectMissingDomainOrOperationWithoutMutatingState() {
        ServiceRequestCommand request = ServiceRequestCommand.parse(")SVC RQ");
        ServiceTableCommand table = ServiceTableCommand.parse(")SVC TBL DOM");

        assertFalse(request.isAccepted());
        assertTrue(request.getErrors().stream().anyMatch(error -> error.contains("domain is missing")));
        assertTrue(request.getErrors().stream().anyMatch(error -> error.contains("operation is missing")));
        assertFalse(table.isAccepted());
        assertTrue(table.getErrors().stream().anyMatch(error -> error.contains("operation is missing")));
    }

    @Test
    void preservesLegacyGrammarFamiliesWithoutMutatingServiceTableState() {
        UsnsMessageEnvelope envelope = UsnsMessageEnvelope.builder()
                .originAddress("KZNY")
                .body("(A0001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)\n"
                        + "(A0002/26 NOTAMC A) KZNY E) CANCEL TEST)\n"
                        + "(1234/26 NOTAMJ A) CYUL E) CANADIAN TEST TIL APRX 2601011200)\n"
                        + ")SVC RQ DOM LOC=KZNY COUNT\n"
                        + ")SVC TBL DOM LST CROSSOVER\n"
                        + "GENOT RWA 1/26 CANCELLATION: 2601011200")
                .build();

        UsnsTransactionParseResult result = new UsnsTransactionSplitter().split(envelope);

        assertTrue(result.isAccepted(), result.getErrors().toString());
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ICAO_NOTAMN
                && t.getRawText().contains("Q) /QRTCA")));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.ICAO_NOTAMC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.CANADIAN_DOMESTIC));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SERVICE_REQUEST));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.SERVICE_TABLE
                && t.getRawText().contains("LST CROSSOVER")));
        assertTrue(result.getTransactions().stream().anyMatch(t -> t.getType() == UsnsTransactionType.GENOT));
    }
}
