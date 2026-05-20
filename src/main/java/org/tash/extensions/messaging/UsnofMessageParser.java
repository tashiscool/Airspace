package org.tash.extensions.messaging;

public class UsnofMessageParser extends UsnsMessageEnvelopeParser {
    @Override
    public UsnsMessageParseResult parse(String raw) {
        return parseNadinLike(raw, false, true);
    }
}
