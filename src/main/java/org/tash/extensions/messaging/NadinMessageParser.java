package org.tash.extensions.messaging;

public class NadinMessageParser extends UsnsMessageEnvelopeParser {
    @Override
    public UsnsMessageParseResult parse(String raw) {
        return parseNadinLike(raw, false, false);
    }
}
