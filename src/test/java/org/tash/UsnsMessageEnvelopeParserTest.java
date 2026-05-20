package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.messaging.NadinMessageParser;
import org.tash.extensions.messaging.UsnsMessageParseResult;
import org.tash.extensions.messaging.UsnsRoutingDecision;
import org.tash.extensions.messaging.WmscrMessageParser;

import static org.junit.jupiter.api.Assertions.*;

class UsnsMessageEnvelopeParserTest {
    @Test
    void parsesNadinEnvelopeAndUsnofRoutingMetadata() {
        String raw = "01GGNC07GP\n"
                + MessageControlCharacters.SOH + "CNS000 300334\n"
                + "GG KDZZNAXX KCNFYNYX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + "!ABC ABC RWY 04 CLSD 1001011200-1001011300"
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;

        UsnsMessageParseResult result = new NadinMessageParser().parse(raw);

        assertTrue(result.isAccepted());
        assertEquals("01", result.getEnvelope().getMrsHeader().getFunctionCode());
        assertEquals("GG", result.getEnvelope().getPriority());
        assertEquals("KDZZNAXX", result.getEnvelope().getToAddresses().get(0));
        assertEquals("300334", result.getEnvelope().getOriginDhm());
        assertEquals("KGPS", result.getEnvelope().getOriginAddress());
        assertEquals("!ABC ABC RWY 04 CLSD 1001011200-1001011300", result.getNormalizedBody());
    }

    @Test
    void rejectsInvalidOriginDateAndDetectsNatRouting() {
        String raw = "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "BADDATE KGPS\n"
                + MessageControlCharacters.STX + "NORTH ATLANTIC ADVISORY TEXT";

        UsnsMessageParseResult result = new NadinMessageParser().parse(raw);

        assertFalse(result.isAccepted());
        assertTrue(result.getErrors().get(0).contains("Origin date-time group"));
        assertEquals(UsnsRoutingDecision.NAT_TRACK, result.getRoutingDecision());
    }

    @Test
    void parsesWmscrPrivilegedAndRejectedCategories() {
        String privileged = "01GGCE00  \n"
                + MessageControlCharacters.EF + "90006 300334\n"
                + "D KDZZNAXX\n"
                + "300334 WMSCR\n"
                + MessageControlCharacters.STX + ")SVC RQ DOM LOC=PDT";
        String rejected = "01GGCE00  \n"
                + MessageControlCharacters.EF + "90008 300334\n"
                + "D KDZZNAXX\n"
                + "300334 WMSCR\n"
                + MessageControlCharacters.STX + "REJECT";

        UsnsMessageParseResult privilegedResult = new WmscrMessageParser().parse(privileged);
        UsnsMessageParseResult rejectedResult = new WmscrMessageParser().parse(rejected);

        assertTrue(privilegedResult.isAccepted());
        assertEquals(UsnsRoutingDecision.PRIVILEGED_REQUEST, privilegedResult.getRoutingDecision());
        assertFalse(rejectedResult.isAccepted());
        assertEquals(UsnsRoutingDecision.REJECTED, rejectedResult.getRoutingDecision());
    }
}
