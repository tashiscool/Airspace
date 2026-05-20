package org.tash.extensions.messaging;

import java.time.Clock;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LegacySampleFixtureExpander {
    private static final Pattern OFFSET = Pattern.compile("%\\?([+-]\\d+)");
    private final Clock clock;

    public LegacySampleFixtureExpander() {
        this(Clock.systemUTC());
    }

    public LegacySampleFixtureExpander(Clock clock) {
        this.clock = clock;
    }

    public List<LegacySampleFixture> expand(String source) {
        List<LegacySampleFixture> fixtures = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        int index = 1;
        for (String line : MessageControlCharacters.normalizeLineEndings(source).split("\n", -1)) {
            String trimmed = line.trim();
            if (trimmed.equals("/")) {
                if (current.length() > 0) {
                    fixtures.add(fixture(index++, current));
                    current.setLength(0);
                }
            } else if (!trimmed.startsWith("*") && !trimmed.startsWith("#")) {
                current.append(line).append('\n');
            }
        }
        if (current.length() > 0) {
            fixtures.add(fixture(index, current));
        }
        return fixtures;
    }

    private LegacySampleFixture fixture(int index, StringBuilder current) {
        return LegacySampleFixture.builder()
                .name("sample-" + index)
                .text(substitute(current.toString()).trim())
                .build();
    }

    private String substitute(String text) {
        ZonedDateTime now = ZonedDateTime.now(clock);
        String result = text
                .replace("%C", now.format(DateTimeFormatter.ofPattern("yyyy")).substring(0, 2))
                .replace("%Y", now.format(DateTimeFormatter.ofPattern("yy")))
                .replace("%M", now.format(DateTimeFormatter.ofPattern("MM")))
                .replace("%D", now.format(DateTimeFormatter.ofPattern("dd")))
                .replace("%h", now.format(DateTimeFormatter.ofPattern("HH")))
                .replace("%m", now.format(DateTimeFormatter.ofPattern("mm")))
                .replace("%s", now.format(DateTimeFormatter.ofPattern("ss")))
                .replace("%J", now.format(DateTimeFormatter.ofPattern("DDD")))
                .replace("%L", now.format(DateTimeFormatter.ofPattern("yyyyMMddHHmm")))
                .replace("%O", now.format(DateTimeFormatter.ofPattern("ddHHmm")));
        Matcher matcher = OFFSET.matcher(result);
        StringBuffer buffer = new StringBuffer();
        while (matcher.find()) {
            int minutes = Integer.parseInt(matcher.group(1));
            matcher.appendReplacement(buffer, ZonedDateTime.now(clock).plusMinutes(minutes)
                    .format(DateTimeFormatter.ofPattern("yyMMddHHmm")));
        }
        matcher.appendTail(buffer);
        return buffer.toString().replace("%%", "%");
    }
}
