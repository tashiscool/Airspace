package org.tash.extensions.messaging.transaction;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Data
@Builder
public class ServiceRequestCommand {
    private boolean accepted;
    private String service;
    private String domain;
    private String operation;
    private String requestFormat;
    private String location;
    private String notamId;
    private String rangeStart;
    private String rangeEnd;
    private boolean count;
    private boolean list;
    private boolean history;
    private boolean all;
    private boolean current;
    private boolean wmscrEcho;
    private boolean privilegedHistoryRequest;
    private List<String> arguments;
    private List<String> warnings;
    private List<String> errors;

    public static ServiceRequestCommand parse(String text) {
        return parseWithService(text, "RQ");
    }

    static ServiceRequestCommand parseWithService(String text, String expectedService) {
        String normalized = (text == null ? "" : text).replaceAll("[()]", " ").trim().replaceAll("\\s+", " ")
                .toUpperCase(Locale.US);
        String[] tokens = normalized.split("\\s+");
        List<String> args = new ArrayList<>(Arrays.asList(tokens));
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        int svc = args.indexOf("SVC");
        if (svc < 0 && !args.isEmpty() && "RQN".equals(args.get(0))) {
            return rqn(args, warnings, errors);
        }
        if (svc < 0 || svc + 1 >= args.size()) {
            errors.add("SVC command marker is missing");
            return ServiceRequestCommand.builder()
                    .accepted(false)
                    .service(expectedService)
                    .arguments(args)
                    .warnings(Collections.unmodifiableList(warnings))
                    .errors(Collections.unmodifiableList(errors))
                    .build();
        }
        String service = args.get(svc + 1);
        String domain = svc + 2 < args.size() ? args.get(svc + 2) : "";
        String operation = svc + 3 < args.size() ? args.get(svc + 3) : "";
        List<String> rest = svc + 4 < args.size() ? new ArrayList<>(args.subList(svc + 4, args.size())) : new ArrayList<>();
        Fields fields = fields(args, rest);
        if (!expectedService.equals(service)) {
            warnings.add("Parsed service " + service + " while expecting " + expectedService);
        }
        return ServiceRequestCommand.builder()
                .accepted(errors.isEmpty())
                .service(service)
                .domain(domain)
                .operation(operation)
                .requestFormat("SVC")
                .location(fields.location)
                .notamId(fields.notamId)
                .rangeStart(fields.rangeStart)
                .rangeEnd(fields.rangeEnd)
                .count(fields.count)
                .list(fields.list)
                .history(fields.history)
                .all(fields.all)
                .current(fields.current)
                .wmscrEcho(normalized.contains("WMSCR"))
                .privilegedHistoryRequest(fields.history)
                .arguments(rest)
                .warnings(Collections.unmodifiableList(warnings))
                .errors(Collections.unmodifiableList(errors))
                .build();
    }

    private static ServiceRequestCommand rqn(List<String> args, List<String> warnings, List<String> errors) {
        Fields fields = fields(args, args.size() > 1 ? args.subList(1, args.size()) : Collections.emptyList());
        String location = args.size() > 1 ? args.get(1) : fields.location;
        if (location == null || location.isEmpty()) {
            errors.add("RQN location is missing");
        }
        return ServiceRequestCommand.builder()
                .accepted(errors.isEmpty())
                .service("RQ")
                .domain("DOM")
                .operation("RQN")
                .requestFormat("RQN")
                .location(location)
                .notamId(fields.notamId)
                .rangeStart(fields.rangeStart)
                .rangeEnd(fields.rangeEnd)
                .count(fields.count)
                .list(fields.list)
                .history(fields.history)
                .all(fields.all)
                .current(fields.current)
                .wmscrEcho(false)
                .privilegedHistoryRequest(fields.history)
                .arguments(args.size() > 2 ? new ArrayList<>(args.subList(2, args.size())) : Collections.emptyList())
                .warnings(Collections.unmodifiableList(warnings))
                .errors(Collections.unmodifiableList(errors))
                .build();
    }

    private static Fields fields(List<String> allTokens, List<String> rest) {
        Fields fields = new Fields();
        for (String token : allTokens) {
            if (token.startsWith("LOC=")) {
                fields.location = token.substring("LOC=".length());
            } else if (token.matches("[A-Z0-9]?\\d{1,5}/\\d{2,4}")) {
                if (fields.notamId == null) {
                    fields.notamId = token;
                } else if (fields.rangeStart == null) {
                    fields.rangeStart = fields.notamId;
                    fields.rangeEnd = token;
                }
            } else if ("COUNT".equals(token) || "CNT".equals(token)) {
                fields.count = true;
            } else if ("LST".equals(token) || "LIST".equals(token)) {
                fields.list = true;
            } else if ("HIST".equals(token) || "HISTORY".equals(token) || "ALL".equals(token)) {
                fields.history = true;
                fields.all = true;
            } else if ("CUR".equals(token) || "CURRENT".equals(token)) {
                fields.current = true;
            }
        }
        if ((fields.location == null || fields.location.isEmpty()) && !rest.isEmpty()) {
            for (String token : rest) {
                if (token.matches("[A-Z]{3,4}")) {
                    fields.location = token;
                    break;
                }
            }
        }
        return fields;
    }

    private static class Fields {
        private String location;
        private String notamId;
        private String rangeStart;
        private String rangeEnd;
        private boolean count;
        private boolean list;
        private boolean history;
        private boolean all;
        private boolean current;
    }
}
