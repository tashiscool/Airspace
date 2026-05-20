package org.tash.extensions.engine.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class SpatialTopologyExtensionRegistry {
    private final Map<SpatialTopologyBackend, SpatialTopologyExtension> extensions =
            new EnumMap<>(SpatialTopologyBackend.class);

    public SpatialTopologyExtensionRegistry() {
        this(defaultExtensions());
    }

    public SpatialTopologyExtensionRegistry(List<SpatialTopologyExtension> extensions) {
        for (SpatialTopologyExtension extension : extensions == null ? Collections.<SpatialTopologyExtension>emptyList() : extensions) {
            if (extension != null) {
                this.extensions.put(extension.backend(), extension);
            }
        }
        this.extensions.putIfAbsent(SpatialTopologyBackend.NATIVE, new NativeSpatialTopologyExtension());
    }

    public static SpatialTopologyExtensionRegistry defaults() {
        return new SpatialTopologyExtensionRegistry();
    }

    public Optional<SpatialTopologyExtension> byBackend(SpatialTopologyBackend backend) {
        if (backend == null) {
            return Optional.empty();
        }
        SpatialTopologyExtension extension = extensions.get(backend);
        return extension != null && extension.isAvailable() ? Optional.of(extension) : Optional.empty();
    }

    public SpatialTopologyExtension bestForPreciseTopology(SpatialTopologyBackend preferred) {
        return byBackend(preferred)
                .filter(SpatialTopologyExtension::supportsPreciseTopology)
                .orElseGet(() -> byBackend(SpatialTopologyBackend.JTS)
                        .filter(SpatialTopologyExtension::supportsPreciseTopology)
                        .orElseGet(() -> extensions.get(SpatialTopologyBackend.NATIVE)));
    }

    public SpatialTopologyExtension bestForDiscreteIndex(SpatialTopologyBackend preferred) {
        return byBackend(preferred)
                .filter(SpatialTopologyExtension::supportsDiscreteIndex)
                .orElseGet(() -> byBackend(SpatialTopologyBackend.H3)
                        .filter(SpatialTopologyExtension::supportsDiscreteIndex)
                        .orElseGet(() -> byBackend(SpatialTopologyBackend.S2)
                                .filter(SpatialTopologyExtension::supportsDiscreteIndex)
                                .orElseGet(() -> extensions.get(SpatialTopologyBackend.NATIVE))));
    }

    public List<SpatialTopologyExtension> availableExtensions() {
        List<SpatialTopologyExtension> available = new ArrayList<>();
        for (SpatialTopologyExtension extension : extensions.values()) {
            if (extension.isAvailable()) {
                available.add(extension);
            }
        }
        return Collections.unmodifiableList(available);
    }

    private static List<SpatialTopologyExtension> defaultExtensions() {
        List<SpatialTopologyExtension> defaults = new ArrayList<>();
        defaults.add(new JtsSpatialTopologyExtension());
        defaults.add(new H3SpatialTopologyExtension());
        defaults.add(new S2SpatialTopologyExtension());
        defaults.add(new NativeSpatialTopologyExtension());
        return defaults;
    }
}
