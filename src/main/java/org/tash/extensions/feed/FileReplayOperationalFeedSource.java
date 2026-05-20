package org.tash.extensions.feed;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.UUID;
import java.util.stream.Stream;

public class FileReplayOperationalFeedSource implements OperationalFeedSource {
    private final String sourceId;
    private final Path path;

    public FileReplayOperationalFeedSource(String sourceId, Path path) {
        this.sourceId = sourceId == null ? "file-replay" : sourceId;
        this.path = path;
    }

    @Override
    public String sourceId() {
        return sourceId;
    }

    @Override
    public OperationalFeedPollResult poll() {
        List<String> diagnostics = new ArrayList<>();
        List<OperationalFeedEnvelope> envelopes = new ArrayList<>();
        if (path == null || !Files.exists(path)) {
            diagnostics.add("Feed path does not exist: " + path);
            return OperationalFeedPollResult.builder().accepted(false).diagnostics(diagnostics).build();
        }
        try {
            List<Path> files = new ArrayList<>();
            if (Files.isDirectory(path)) {
                try (Stream<Path> stream = Files.list(path)) {
                    stream.filter(Files::isRegularFile).sorted(Comparator.comparing(Path::toString)).forEach(files::add);
                }
            } else {
                files.add(path);
            }
            for (Path file : files) {
                String raw = Files.readString(file);
                envelopes.add(OperationalFeedEnvelope.builder()
                        .id(UUID.nameUUIDFromBytes((sourceId + ":" + file).getBytes()).toString())
                        .sourceId(sourceId)
                        .type(typeFromName(file.getFileName().toString(), raw))
                        .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .rawPayload(raw)
                        .build());
            }
        } catch (IOException ex) {
            diagnostics.add("Unable to read feed path " + path + ": " + ex.getMessage());
        }
        return OperationalFeedPollResult.builder()
                .accepted(diagnostics.isEmpty())
                .envelopes(envelopes)
                .diagnostics(diagnostics)
                .build();
    }

    private OperationalFeedType typeFromName(String name, String raw) {
        String text = ((name == null ? "" : name) + " " + (raw == null ? "" : raw)).toUpperCase(Locale.US);
        if (text.contains("USNS") || text.contains("NADIN") || text.contains("WMSCR") || text.contains("!")) return OperationalFeedType.USNS;
        if (text.contains("CARF") || text.contains("ALTRV") || text.contains("APREQ")) return OperationalFeedType.CARF_ALTRV;
        if (text.contains("PIREP") || text.contains(" UUA ") || text.contains("/OV") || text.contains("/TB")) return OperationalFeedType.PIREP;
        if (text.contains("METAR") || text.contains("TAF") || text.contains("SIGMET") || text.contains("AIRMET") || text.contains("CWAP")) return OperationalFeedType.WEATHER;
        if (text.contains("NOTAM") || text.contains("Q)")) return OperationalFeedType.NOTAM;
        return OperationalFeedType.UNKNOWN;
    }
}
