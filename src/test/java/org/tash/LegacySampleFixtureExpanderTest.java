package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.messaging.LegacySampleFixture;
import org.tash.extensions.messaging.LegacySampleFixtureExpander;

import java.time.Clock;
import java.time.Instant;
import java.time.ZoneOffset;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class LegacySampleFixtureExpanderTest {
    @Test
    void expandsLegacySampleFilesDeterministically() {
        LegacySampleFixtureExpander expander = new LegacySampleFixtureExpander(
                Clock.fixed(Instant.parse("2010-03-02T13:04:05Z"), ZoneOffset.UTC));

        List<LegacySampleFixture> fixtures = expander.expand("* ignored\n"
                + "!ABC %Y%M%D%h%m %C %s %J %L %O %?+20 %%\n"
                + "/\n"
                + "# ignored\n"
                + "!DEF %?-10\n");

        assertEquals(2, fixtures.size());
        assertEquals("!ABC 1003021304 20 05 061 201003021304 021304 1003021324 %",
                fixtures.get(0).getText());
        assertEquals("!DEF 1003021254", fixtures.get(1).getText());
    }
}
