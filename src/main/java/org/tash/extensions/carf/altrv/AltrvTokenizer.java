package org.tash.extensions.carf.altrv;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class AltrvTokenizer {
    public List<AltrvToken> tokenize(String raw) {
        List<AltrvToken> tokens = new ArrayList<>();
        String text = raw == null ? "" : raw.replace('\r', '\n');
        int i = 0;
        while (i < text.length()) {
            char ch = text.charAt(i);
            if (Character.isWhitespace(ch)) {
                i++;
                continue;
            }
            int start = i;
            if (Character.isLetterOrDigit(ch)) {
                while (i < text.length() && isTokenCharacter(text.charAt(i))) {
                    i++;
                }
                String value = text.substring(start, i).toUpperCase(Locale.US);
                tokens.add(AltrvToken.builder()
                        .type(classify(value, text, start, i))
                        .text(value)
                        .offset(start)
                        .endOffset(i)
                        .sourceSpan(span(start, i, value))
                        .build());
                continue;
            }
            tokens.add(AltrvToken.builder()
                    .type(AltrvTokenType.PUNCTUATION)
                    .text(Character.toString(ch))
                    .offset(start)
                    .endOffset(start + 1)
                    .sourceSpan(span(start, start + 1, Character.toString(ch)))
                    .build());
            i++;
        }
        return tokens;
    }

    private AltrvSourceSpan span(int start, int end, String value) {
        return AltrvSourceSpan.builder()
                .startOffset(start)
                .endOffset(end)
                .text(value)
                .build();
    }

    private boolean isTokenCharacter(char ch) {
        return Character.isLetterOrDigit(ch) || ch == '/' || ch == '-';
    }

    private AltrvTokenType classify(String value, String fullText, int start, int end) {
        if (value.matches("[A-G]") && end < fullText.length() && fullText.charAt(end) == '.') {
            return AltrvTokenType.SECTION_LABEL;
        }
        if (value.matches("\\d{4}[NS]")) {
            return AltrvTokenType.COORDINATE_LATITUDE;
        }
        if (value.matches("\\d{4,5}[EW]")) {
            return AltrvTokenType.COORDINATE_LONGITUDE;
        }
        if (value.matches("\\d{4}")) {
            return AltrvTokenType.TIME;
        }
        if (value.matches("FL\\d{3}B\\d{3}")) {
            return AltrvTokenType.FLIGHT_LEVEL_RANGE;
        }
        if (value.matches("\\d+")) {
            return AltrvTokenType.NUMBER;
        }
        return AltrvTokenType.WORD;
    }
}
