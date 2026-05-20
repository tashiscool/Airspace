package org.tash.extensions.feed;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class FileReplayOperationalFeedSourceTest {
    @TempDir
    Path tempDir;

    @Test
    void replaysDirectoryInStableOrderAndClassifiesOperationalPayloads() throws Exception {
        Files.writeString(tempDir.resolve("01-weather.txt"), "METAR KJFK 200000Z 18012KT 2SM RA BKN010");
        Files.writeString(tempDir.resolve("02-pirep.txt"), "UA /OV JFK090020 /TM 2000 /FL240 /TB MOD");
        Files.writeString(tempDir.resolve("03-usns.txt"), "!DCA DCA RWY 01 CLSD WEF 201200-201300");

        OperationalFeedPollResult result = new FileReplayOperationalFeedSource("replay-test", tempDir).poll();

        assertTrue(result.isAccepted());
        assertEquals("replay-test", new FileReplayOperationalFeedSource("replay-test", tempDir).sourceId());
        assertEquals(3, result.getEnvelopes().size());
        assertEquals(List.of(
                "METAR KJFK 200000Z 18012KT 2SM RA BKN010",
                "UA /OV JFK090020 /TM 2000 /FL240 /TB MOD",
                "!DCA DCA RWY 01 CLSD WEF 201200-201300"), result.getEnvelopes().stream()
                .map(OperationalFeedEnvelope::getRawPayload)
                .collect(Collectors.toList()));

        Map<String, OperationalFeedType> byPayload = result.getEnvelopes().stream()
                .collect(Collectors.toMap(OperationalFeedEnvelope::getRawPayload, OperationalFeedEnvelope::getType));
        assertEquals(OperationalFeedType.WEATHER, byPayload.get("METAR KJFK 200000Z 18012KT 2SM RA BKN010"));
        assertEquals(OperationalFeedType.PIREP, byPayload.get("UA /OV JFK090020 /TM 2000 /FL240 /TB MOD"));
        assertEquals(OperationalFeedType.USNS, byPayload.get("!DCA DCA RWY 01 CLSD WEF 201200-201300"));
        assertTrue(result.getEnvelopes().stream().allMatch(envelope -> envelope.getReceivedAt() != null));
    }

    @Test
    void reportsMissingReplayPathAsRejectedDiagnostic() {
        OperationalFeedPollResult result = new FileReplayOperationalFeedSource(null, tempDir.resolve("missing")).poll();

        assertFalse(result.isAccepted());
        assertEquals("file-replay", new FileReplayOperationalFeedSource(null, tempDir).sourceId());
        assertTrue(result.getEnvelopes().isEmpty());
        assertEquals(1, result.getDiagnostics().size());
        assertTrue(result.getDiagnostics().get(0).contains("does not exist"));
    }
}
