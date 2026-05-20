package org.tash.extensions.evaluation;

import lombok.Builder;
import lombok.Data;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class LegacyJavaArtifactAnalyzer {
    private static final Pattern IMPORT = Pattern.compile("(?m)^import\\s+([^;]+);");

    public Analysis analyze(Path sourceFile) throws IOException {
        String source = new String(Files.readAllBytes(sourceFile), "UTF-8");
        Set<String> imports = new LinkedHashSet<>();
        Set<String> externalFamilies = new LinkedHashSet<>();
        Matcher matcher = IMPORT.matcher(source);
        while (matcher.find()) {
            String importName = matcher.group(1).trim();
            imports.add(importName);
            if (importName.startsWith("com.luciad")) {
                externalFamilies.add("com.luciad");
            } else if (importName.startsWith("gov.faa")) {
                externalFamilies.add("gov.faa");
            } else if (importName.startsWith("javax.swing") || importName.startsWith("java.awt")) {
                externalFamilies.add("ui");
            } else if (importName.startsWith("javax.xml.bind")) {
                externalFamilies.add("jaxb");
            }
        }
        return Analysis.builder()
                .sourceFile(sourceFile)
                .imports(imports)
                .externalFamilies(externalFamilies)
                .copyCandidate(externalFamilies.isEmpty())
                .build();
    }

    @Data
    @Builder
    public static class Analysis {
        private Path sourceFile;
        private Set<String> imports;
        private Set<String> externalFamilies;
        private boolean copyCandidate;
    }
}
