package org.tash.extensions.agentic.mcp;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class McpServerDefinition {
    private String id;
    private String name;
    private String description;
    private boolean enabled;
    private boolean external;
    private boolean local;
    private boolean setupRequired;
    private boolean credentialsRequired;
    private String transport;
    private String version;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
