package org.tash.extensions.messaging;

public class WmscrMessageParser extends UsnsMessageEnvelopeParser {
    @Override
    public UsnsMessageParseResult parse(String raw) {
        return parseNadinLike(raw, true, false);
    }
}
