package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class WaypointFileResolver extends MapWaypointResolver {
    public WaypointFileResolver(Map<String, GeoCoordinate> waypoints) {
        super(waypoints);
    }

    public static WaypointFileResolver fromDelimited(Path path) throws IOException {
        Map<String, GeoCoordinate> waypoints = new LinkedHashMap<>();
        List<String> lines = Files.readAllLines(path);
        for (String rawLine : lines) {
            String line = stripComment(rawLine).trim();
            if (line.isEmpty()) {
                continue;
            }
            String[] fields = line.contains(",") ? line.split("\\s*,\\s*") : line.split("\\s+");
            if (fields.length < 3 || isHeader(fields)) {
                continue;
            }
            waypoints.put(fields[0], GeoCoordinate.builder()
                    .latitude(Double.parseDouble(fields[1]))
                    .longitude(Double.parseDouble(fields[2]))
                    .altitude(fields.length >= 4 ? Double.parseDouble(fields[3]) : 0.0)
                    .build());
        }
        return new WaypointFileResolver(waypoints);
    }

    private static String stripComment(String line) {
        int index = line.indexOf('#');
        return index >= 0 ? line.substring(0, index) : line;
    }

    private static boolean isHeader(String[] fields) {
        String first = fields[0].toLowerCase(Locale.US);
        return first.equals("id") || first.equals("identifier") || first.equals("name") || first.equals("fix");
    }
}
