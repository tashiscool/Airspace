package org.tash.extensions.feed;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class Flightradar24TrafficReplayAdapterTest {
    @Test
    void mapsRecordedFr24PositionsIntoTrafficReplayBundle() {
        Flightradar24TrafficReplayAdapter adapter = new Flightradar24TrafficReplayAdapter(false, "", null, null, 50);
        FlightAwarenessFeedPollRequest request = new FlightAwarenessFeedPollRequest();
        request.setSourceId("fr24-recorded-test");
        request.setRawResponseJson("""
                {
                  "data": [
                    {
                      "fr24_id": "34ABC123",
                      "flight": "BAW1",
                      "callsign": "BAW1",
                      "lat": 40.6413,
                      "lon": -73.7781,
                      "track": 70,
                      "alt": 0,
                      "gspeed": 0,
                      "timestamp": "2026-07-12T12:00:00Z",
                      "source": "ADSB",
                      "type": "B77W",
                      "operating_as": "BAW",
                      "orig_icao": "KJFK",
                      "dest_icao": "EGLL"
                    },
                    {
                      "fr24_id": "34ABC123",
                      "flight": "BAW1",
                      "callsign": "BAW1",
                      "lat": 40.95,
                      "lon": -72.60,
                      "track": 75,
                      "alt": 16000,
	                      "gspeed": 390,
	                      "timestamp": "2026-07-12T12:15:00Z",
	                      "source": "SAT_ADSB",
	                      "type": "B77W",
	                      "operating_as": "BAW",
                      "orig_icao": "KJFK",
                      "dest_icao": "EGLL"
                    }
                  ]
                }
                """);

        FlightAwarenessFeedPollResult result = adapter.poll(request);

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertFalse(result.isLiveCallAttempted());
        assertEquals("AUTHENTICATED_EXTERNAL_REPLAY_SAMPLE", result.getSourceMode());
        assertNotNull(result.getTrafficReplay());
        assertEquals("FLIGHTRADAR24", result.getTrafficReplay().getProviderFamily());
        assertEquals(1, result.getTrafficReplay().getFlightPlans().size());
        assertEquals(2, result.getTrafficReplay().getPositions().size());
        assertEquals(1, result.getTrafficReplay().getAirportDemand().size());
        assertEquals(1, result.getTrafficReplay().getSectorDemand().size());
        assertEquals("KJFK", result.getTrafficReplay().getAirportDemand().get(0).getAirportId());
        assertEquals("ADS_B", result.getTrafficReplay().getPositions().get(0).getTrackingSourceType());
        assertEquals("SATELLITE_ADSB", result.getTrafficReplay().getPositions().get(1).getTrackingSourceType());
        assertTrue(result.getTrafficReplay().getTrackingSourceTypes().contains("ADS_B"));
        assertTrue(result.getTrafficReplay().getTrackingSourceTypes().contains("SATELLITE_ADSB"));
        assertTrue(result.getTrafficReplay().getProviderTransports().contains("RECORDED_JSON"));
        assertTrue(result.getTrafficReplay().getProviderTransports().contains("MCP_COMPATIBLE_SHAPE"));
        assertTrue(result.getTrafficReplay().getCoverageWarnings().stream()
                .anyMatch(warning -> warning.contains("Satellite ADS-B")));
        assertTrue(result.getTrafficReplay().getAssumptions().stream()
                .anyMatch(assumption -> assumption.contains("not authoritative FAA")));
    }

    @Test
    void livePollingIsSetupRequiredWhenDisabled() {
        Flightradar24TrafficReplayAdapter adapter = new Flightradar24TrafficReplayAdapter(false, "", null, null, 50);
        FlightAwarenessFeedPollRequest request = new FlightAwarenessFeedPollRequest();
        request.setBounds("41.0,40.5,-74.5,-73.5");

        FlightAwarenessFeedPollResult result = adapter.poll(request);

        assertFalse(result.isAccepted());
        assertTrue(result.isSetupRequired());
        assertFalse(result.isLiveCallAttempted());
        assertTrue(result.getDiagnostics().stream().anyMatch(message -> message.contains("disabled")));
    }
}
