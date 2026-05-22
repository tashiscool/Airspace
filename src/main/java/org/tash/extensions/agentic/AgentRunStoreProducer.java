package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.enterprise.inject.Produces;
import org.eclipse.microprofile.config.inject.ConfigProperty;

import java.nio.file.Path;
import java.util.Optional;

@ApplicationScoped
public class AgentRunStoreProducer {
    @Produces
    @ApplicationScoped
    public AgentRunStore agentRunStore(@ConfigProperty(name = "airspace.agentic.store.path") Optional<String> path) {
        if (path.isPresent() && !path.get().trim().isEmpty()) {
            return new JsonFileAgentRunStore(Path.of(path.get().trim()));
        }
        return new InMemoryAgentRunStore();
    }
}
