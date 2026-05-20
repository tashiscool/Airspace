package org.tash.extensions.carf.altrv;

import java.util.Collections;
import java.util.List;

public class AltrvSectionParser {
    private final AltrvParser parser;

    public AltrvSectionParser() {
        this(new AltrvParser());
    }

    public AltrvSectionParser(AltrvParser parser) {
        this.parser = parser;
    }

    public AltrvParseResult parse(String raw) {
        return parser.parse(raw);
    }

    public List<AltrvSectionResult> parseSections(String raw) {
        AltrvParseResult result = parse(raw);
        return result == null || result.getSectionResults() == null
                ? Collections.emptyList()
                : result.getSectionResults();
    }
}
