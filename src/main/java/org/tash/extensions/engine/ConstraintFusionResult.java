package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class ConstraintFusionResult {
    @Builder.Default
    private final List<OperationalConstraint> constraints = new ArrayList<>();

    public List<OperationalConstraint> getConstraints() {
        return Collections.unmodifiableList(constraints);
    }
}
