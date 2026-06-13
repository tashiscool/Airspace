package org.tash.extensions.agentic.mcp;

import java.util.List;
import java.util.Optional;

public interface McpToolRegistry {
    List<McpServerDefinition> servers();

    Optional<McpServerDefinition> server(String serverId);

    List<McpToolDescriptor> tools(String serverId);

    Optional<McpToolDescriptor> tool(String serverId, String toolId);
}
