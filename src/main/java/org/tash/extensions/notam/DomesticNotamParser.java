package org.tash.extensions.notam;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Modern parser for the domestic NOTAM shape described by the legacy Dom1.ycc
 * Visual Parse++ grammar from the 2010 parser tester files.
 */
public class DomesticNotamParser {
    private final DomesticNotamContractionDictionary contractionDictionary;
    private final DomesticQCodeClassifier qCodeClassifier = new DomesticQCodeClassifier();
    private final GlobalAccountKeywordResolver globalAccountKeywordResolver;

    public DomesticNotamParser() {
        this(new GlobalAccountKeywordResolver());
    }

    public DomesticNotamParser(GlobalAccountKeywordResolver globalAccountKeywordResolver) {
        this(new DomesticNotamContractionDictionary(), globalAccountKeywordResolver);
    }

    public DomesticNotamParser(DomesticNotamContractionDictionary contractionDictionary,
                               GlobalAccountKeywordResolver globalAccountKeywordResolver) {
        this.contractionDictionary = contractionDictionary;
        this.globalAccountKeywordResolver = globalAccountKeywordResolver;
    }

    private static final Set<String> KEYWORDS = new HashSet<>(Arrays.asList(
            "AD", "AIRSPACE", "APRON", "COM", "NAV", "OBST", "RAMP", "RWY", "SVC", "TWY",
            "ODP", "SID", "STAR", "CHART", "DATA", "IAP", "VFP", "ROUTE", "SPECIAL", "(O)"
    ));
    private static final Pattern DATE_RANGE =
            Pattern.compile("\\b(\\d{10}|\\d{12})\\s*-\\s*((?:\\d{10}|\\d{12})|PERM)(EST)?\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern WEF_TIL =
            Pattern.compile("\\bWEF\\s+(\\d{10}|\\d{12})\\b(?:.*?\\bTIL\\s+(APRX\\s+)?((?:\\d{10}|\\d{12})|PERM)(EST)?)?",
                    Pattern.CASE_INSENSITIVE | Pattern.DOTALL);
    private static final Pattern TIL_ONLY =
            Pattern.compile("\\bTIL\\s+(APRX\\s+)?((?:\\d{10}|\\d{12})|PERM)(EST)?\\b",
                    Pattern.CASE_INSENSITIVE);
    private static final Pattern MALFORMED_WEF =
            Pattern.compile("\\bWEF\\s+(?!\\d{10}\\b|\\d{12}\\b)(\\S+)", Pattern.CASE_INSENSITIVE);
    private static final Pattern MALFORMED_SERVICE_RVR =
            Pattern.compile("^\\s*(?:RWY\\s+)?[0-9]{1,2}[A-Z]?(?:/[0-9]{1,2}[A-Z]?)?\\s+RVR[TM R]?\\b",
                    Pattern.CASE_INSENSITIVE);
    private static final Pattern MALFORMED_RUNWAY_NOW =
            Pattern.compile("^\\s*NOW\\s+[0-9]{1,2}[A-Z]?(?:/[0-9]{1,2}[A-Z]?)?\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern MALFORMED_RUNWAY_WARNING_BARE_OBJECT =
            Pattern.compile("\\b(?:WARNING|PAEW|WIP)\\b.*\\b(?:OF|ABV|BLW|BELOW|BTN|BTWN)\\s+[0-9]{1,2}[A-Z]?(?:/[0-9]{1,2}[A-Z]?)?\\s*/\\s*(?:TWY|TAXIWAY)\\b",
                    Pattern.CASE_INSENSITIVE);
    private static final Pattern MALFORMED_NAV_RUNWAY_LANDING_AID =
            Pattern.compile("^\\s*RWY\\s+(?:ILS|MLS|LDA|LOC|LLZ|DME|GLIDEPATH|GLIDE\\s+PATH)\\b",
                    Pattern.CASE_INSENSITIVE);
    private static final Pattern COMMENT = Pattern.compile("\\^\\^\\s*(.*)$", Pattern.DOTALL);

    public DomesticNotamRecord parse(String rawText) {
        return parseInternal(rawText).record;
    }

    public DomesticNotamParseResult parseDetailed(String rawText) {
        try {
            ParseOutcome outcome = parseInternal(rawText);
            List<String> contractions = classifyContractions(outcome.record.getText());
            DomesticNotamSemanticClassification semantic = qCodeClassifier.semantic(outcome.record, contractions);
            List<String> warnings = new ArrayList<>(outcome.warnings);
            warnings.addAll(semantic.getWarnings());
            return DomesticNotamParseResult.builder()
                    .accepted(true)
                    .record(outcome.record)
                    .inferredKeyword(outcome.inferredKeyword)
                    .contractionClassification(outcome.record.getKeyword())
                    .q23(semantic.getQ23())
                    .q45(semantic.getQ45())
                    .qCodeReason(semantic.getReducerName())
                    .semanticFacilityFamily(semantic.getFacilityFamily())
                    .semanticCondition(semantic.getCondition())
                    .semanticAction(semantic.getAction())
                    .reducerRuleId(semantic.getReducerRuleId())
                    .reducerName(semantic.getReducerName())
                    .semanticClassification(semantic)
                    .recognizedContractions(contractions)
                    .warnings(warnings)
                    .build();
        } catch (IllegalArgumentException ex) {
            return DomesticNotamParseResult.builder()
                    .accepted(false)
                    .rejectionReason(ex.getMessage())
                    .recognizedContractions(new ArrayList<>())
                    .warnings(new ArrayList<>())
                    .build();
        }
    }

    private ParseOutcome parseInternal(String rawText) {
        if (rawText == null || rawText.trim().isEmpty()) {
            throw new IllegalArgumentException("Domestic NOTAM text is required");
        }
        List<String> warnings = new ArrayList<>();

        String text = rawText.replace("\r", "").trim();
        if (!text.startsWith("!")) {
            throw new IllegalArgumentException("Domestic NOTAM must start with '!'");
        }

        String comment = null;
        Matcher commentMatcher = COMMENT.matcher(text);
        if (commentMatcher.find()) {
            comment = commentMatcher.group(1).trim();
            text = text.substring(0, commentMatcher.start()).trim();
        }

        String[] tokens = text.substring(1).trim().split("\\s+");
        if (tokens.length < 2) {
            throw new IllegalArgumentException("Domestic NOTAM is missing accountability/location");
        }

        String accountability = tokens[0];
        int index = 1;
        DomesticNotamRecord.Type type = DomesticNotamRecord.Type.NEW;
        String notamNumber = null;
        String cancelNumber = null;

        if (isCancelNumber(tokens[index])) {
            type = DomesticNotamRecord.Type.CANCEL;
            cancelNumber = tokens[index++];
            String cancelComment = join(tokens, index);
            return new ParseOutcome(DomesticNotamRecord.builder()
                    .type(type)
                    .accountability(accountability)
                    .cancelNumber(cancelNumber)
                    .text(emptyToNull(cancelComment))
                    .comment(comment)
                    .build(), null, warnings);
        }

        if (isNotamNumber(tokens[index])) {
            type = DomesticNotamRecord.Type.EDIT;
            notamNumber = tokens[index++];
        }

        if (index >= tokens.length) {
            throw new IllegalArgumentException("Domestic NOTAM is missing location");
        }
        String location = tokens[index++];

        boolean unofficial = false;
        if (index < tokens.length && "(U)".equalsIgnoreCase(tokens[index])) {
            unofficial = true;
            index++;
        }

        if (index >= tokens.length) {
            throw new IllegalArgumentException("Domestic NOTAM is missing keyword");
        }
        String keyword = tokens[index].toUpperCase();
        String inferredKeyword = null;
        if (KEYWORDS.contains(keyword)) {
            index++;
        } else if (isCanadianCrossoverWithoutKeyword(accountability, location, type)) {
            keyword = null;
        } else if (isGlobalAccount(accountability)) {
            inferredKeyword = defaultKeywordForGlobalAccount(accountability);
            keyword = inferredKeyword;
            warnings.add("Keyword " + keyword + " inserted for global account " + accountability);
        } else {
            throw new IllegalArgumentException("Unsupported domestic NOTAM keyword: " + keyword);
        }

        String body = join(tokens, index);
        rejectMalformedEffectiveDuration(body);
        rejectMalformedServiceRvr(keyword, body);
        rejectMalformedRunwayObject(keyword, body);
        rejectMalformedNavRunwayLandingAid(keyword, body);
        rejectAirportSnowWithoutRunwayContext(keyword, body);
        TimeWindow timeWindow = extractTimeWindow(body);
        String cleanBody = removeTimeWindow(body).trim();

        return new ParseOutcome(DomesticNotamRecord.builder()
                .type(type)
                .accountability(accountability)
                .notamNumber(notamNumber)
                .location(location)
                .keyword(keyword)
                .unofficial(unofficial)
                .text(emptyToNull(cleanBody))
                .effectiveStart(timeWindow.start)
                .effectiveEnd(timeWindow.end)
                .estimatedEnd(timeWindow.estimated)
                .comment(comment)
                .build(), inferredKeyword, warnings);
    }

    private void rejectMalformedEffectiveDuration(String body) {
        Matcher malformedWef = MALFORMED_WEF.matcher(body);
        if (malformedWef.find()) {
            throw new IllegalArgumentException("Invalid WEF date: " + malformedWef.group(1));
        }
    }

    private void rejectMalformedServiceRvr(String keyword, String body) {
        if (!"SVC".equalsIgnoreCase(keyword == null ? "" : keyword)) {
            return;
        }
        String normalized = body == null ? "" : body.replaceAll("\\s+", " ").trim();
        Matcher malformed = MALFORMED_SERVICE_RVR.matcher(normalized);
        if (malformed.find() && !normalized.toUpperCase().startsWith("RWY ")) {
            throw new IllegalArgumentException("Invalid SVC RVR NOTAM: runway visual range equipment must identify runway as 'RWY <id> RVR ...'");
        }
    }

    private void rejectMalformedRunwayObject(String keyword, String body) {
        if (!"RWY".equalsIgnoreCase(keyword == null ? "" : keyword)) {
            return;
        }
        String normalized = body == null ? "" : body.replaceAll("\\s+", " ").trim();
        if (MALFORMED_RUNWAY_NOW.matcher(normalized).find()) {
            throw new IllegalArgumentException("Invalid runway NOTAM: state-change rows must identify runway as 'RWY <id>', not bare 'NOW <id>'");
        }
        if (MALFORMED_RUNWAY_WARNING_BARE_OBJECT.matcher(normalized).find()) {
            throw new IllegalArgumentException("Invalid runway NOTAM: warning rows must identify each runway/taxiway object explicitly");
        }
    }

    private void rejectMalformedNavRunwayLandingAid(String keyword, String body) {
        if (!"NAV".equalsIgnoreCase(keyword == null ? "" : keyword)) {
            return;
        }
        String normalized = body == null ? "" : body.replaceAll("\\s+", " ").trim();
        if (MALFORMED_NAV_RUNWAY_LANDING_AID.matcher(normalized).find()) {
            throw new IllegalArgumentException("Invalid NAV NOTAM: landing aid rows must name the landing aid before runway context");
        }
    }

    private void rejectAirportSnowWithoutRunwayContext(String keyword, String body) {
        if (!"AD".equalsIgnoreCase(keyword == null ? "" : keyword)) {
            return;
        }
        String normalized = (" " + (body == null ? "" : body).toUpperCase().replaceAll("[^A-Z0-9]+", " ") + " ")
                .replaceAll("\\s+", " ");
        boolean hasSurfaceCondition = normalized.matches(".*\\b(SNOW|SN|ICE|SLUSH|FROST|BERM|WINDROWS|MU|BA)\\b.*");
        if (hasSurfaceCondition && !normalized.contains(" RWY ")) {
            throw new IllegalArgumentException("Snow/surface condition NOTAM must include runway context");
        }
    }

    private TimeWindow extractTimeWindow(String body) {
        Matcher compact = DATE_RANGE.matcher(body);
        if (compact.find()) {
            ZonedDateTime start = parseDateTime(compact.group(1));
            ZonedDateTime end = parseEndDateTime(compact.group(2), start);
            return new TimeWindow(start, end, compact.group(3) != null);
        }

        Matcher explicit = WEF_TIL.matcher(body);
        if (explicit.find()) {
            ZonedDateTime start = parseDateTime(explicit.group(1));
            ZonedDateTime end = explicit.group(3) == null ? null : parseEndDateTime(explicit.group(3), start);
            return new TimeWindow(start, end, explicit.group(2) != null || explicit.group(4) != null);
        }

        Matcher tilOnly = TIL_ONLY.matcher(body);
        if (tilOnly.find()) {
            ZonedDateTime end = "PERM".equalsIgnoreCase(tilOnly.group(2)) ? null : parseDateTime(tilOnly.group(2));
            return new TimeWindow(null, end, tilOnly.group(1) != null || tilOnly.group(3) != null);
        }

        return new TimeWindow(null, null, false);
    }

    private String removeTimeWindow(String body) {
        String result = DATE_RANGE.matcher(body).replaceAll("");
        result = WEF_TIL.matcher(result).replaceAll("");
        result = TIL_ONLY.matcher(result).replaceAll("");
        result = result.replaceAll("(?i)\\bWEF\\s*$", "");
        return result.replaceAll("\\s+", " ");
    }

    private ZonedDateTime parseDateTime(String value) {
        int year = 2000 + Integer.parseInt(value.substring(0, 2));
        int month = Integer.parseInt(value.substring(2, 4));
        int day = Integer.parseInt(value.substring(4, 6));
        int hour = Integer.parseInt(value.substring(6, 8));
        int minute = Integer.parseInt(value.substring(8, 10));
        int second = value.length() >= 12 ? Integer.parseInt(value.substring(10, 12)) : 0;
        if (hour == 24) {
            hour = 23;
            minute = 59;
            second = 0;
        }
        return ZonedDateTime.of(year, month, day, hour, minute, second, 0, ZoneOffset.UTC);
    }

    private ZonedDateTime parseEndDateTime(String value, ZonedDateTime start) {
        if ("PERM".equalsIgnoreCase(value)) {
            return start.plusYears(100);
        }
        return parseDateTime(value);
    }

    private boolean isNotamNumber(String token) {
        return token.matches("\\d+/\\d+");
    }

    private boolean isCancelNumber(String token) {
        return token.matches("C\\d+/\\d+");
    }

    private boolean isCanadianCrossoverWithoutKeyword(String accountability,
                                                       String location,
                                                       DomesticNotamRecord.Type type) {
        return type == DomesticNotamRecord.Type.EDIT
                && (accountability.toUpperCase().startsWith("C") || location.toUpperCase().startsWith("C"));
    }

    private boolean isGlobalAccount(String accountability) {
        return globalAccountKeywordResolver.isGlobalAccount(accountability);
    }

    private String defaultKeywordForGlobalAccount(String accountability) {
        return globalAccountKeywordResolver.defaultKeyword(accountability);
    }

    private String join(String[] tokens, int start) {
        StringBuilder builder = new StringBuilder();
        for (int i = start; i < tokens.length; i++) {
            if (builder.length() > 0) {
                builder.append(' ');
            }
            builder.append(tokens[i]);
        }
        return builder.toString();
    }

    private String emptyToNull(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private List<String> classifyContractions(String text) {
        List<String> classifications = new ArrayList<>();
        if (text == null) {
            return classifications;
        }
        for (String token : text.split("\\s+")) {
            for (String part : token.split("[^A-Za-z0-9]+")) {
                if (part.isEmpty()) {
                    continue;
                }
                contractionDictionary.classify(part)
                        .ifPresent(classification -> classifications.add(part + "=" + classification));
            }
        }
        return classifications;
    }

    private static class TimeWindow {
        private final ZonedDateTime start;
        private final ZonedDateTime end;
        private final boolean estimated;

        private TimeWindow(ZonedDateTime start, ZonedDateTime end, boolean estimated) {
            this.start = start;
            this.end = end;
            this.estimated = estimated;
        }
    }

    private static class ParseOutcome {
        private final DomesticNotamRecord record;
        private final String inferredKeyword;
        private final List<String> warnings;

        private ParseOutcome(DomesticNotamRecord record, String inferredKeyword, List<String> warnings) {
            this.record = record;
            this.inferredKeyword = inferredKeyword;
            this.warnings = warnings;
        }
    }
}
