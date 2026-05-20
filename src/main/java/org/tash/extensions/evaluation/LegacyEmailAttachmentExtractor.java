package org.tash.extensions.evaluation;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Base64;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LegacyEmailAttachmentExtractor {
    public Optional<byte[]> attachment(Path emlPath, String filename) throws IOException {
        String message = new String(Files.readAllBytes(emlPath), StandardCharsets.ISO_8859_1);
        Pattern part = Pattern.compile("(?is)Content-[^\\r\\n]+.*?(?:name|filename)=\""
                + Pattern.quote(filename)
                + "\".*?\\r?\\n\\r?\\n(.*?)(?:\\r?\\n--[^\\r\\n]+|\\z)");
        Matcher matcher = part.matcher(message);
        if (!matcher.find()) {
            return Optional.empty();
        }
        String payload = matcher.group(1)
                .replaceAll("(?m)^Content-[^\\n]+\\r?\\n", "")
                .replaceAll("\\s+", "");
        if (payload.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(Base64.getMimeDecoder().decode(payload));
    }
}
