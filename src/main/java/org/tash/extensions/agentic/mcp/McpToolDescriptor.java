package org.tash.extensions.agentic.mcp;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.agentic.AgenticRiskProfile;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class McpToolDescriptor {
    private String id;
    private String serverId;
    private String name;
    private String description;
    private String version;
    private boolean enabled;
    private boolean external;
    private boolean setupRequired;
    private boolean credentialsRequired;
    private McpSideEffectLevel sideEffectLevel;
    private AgenticRiskProfile riskProfile;
    @Builder.Default
    private List<String> requiredArguments = new ArrayList<>();
    @Builder.Default
    private Map<String, Object> argumentSchema = new LinkedHashMap<>();
    @Builder.Default
    private List<String> sourceFamilies = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
