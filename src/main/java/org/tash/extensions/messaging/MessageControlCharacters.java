package org.tash.extensions.messaging;

public final class MessageControlCharacters {
    public static final char SOH = 0x01;
    public static final char STX = 0x02;
    public static final char ETX = 0x03;
    public static final char LF = 0x0A;
    public static final char VT = 0x0B;
    public static final char CR = 0x0D;
    public static final char RS = 0x1E;
    public static final char GS = 0x1D;
    public static final char EF = 0xEF;

    private MessageControlCharacters() {
    }

    public static String normalizeLineEndings(String text) {
        return text == null ? "" : text.replace("\r\n", "\n").replace('\r', '\n');
    }

    public static String stripBodyTerminators(String text) {
        return text == null ? "" : text.replace(String.valueOf(VT), "")
                .replace(String.valueOf(ETX), "")
                .trim();
    }

    public static String stripLeadingControls(String text) {
        if (text == null) {
            return "";
        }
        int index = 0;
        while (index < text.length()) {
            char ch = text.charAt(index);
            if (ch == SOH || ch == STX || ch == GS || ch == EF || Character.isISOControl(ch) && ch != '\n') {
                index++;
            } else {
                break;
            }
        }
        return text.substring(index).trim();
    }
}
