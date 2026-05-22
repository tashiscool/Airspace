package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class StormLifecycleObservation {
    private String cellId;
    private String phase;
    private double growthRate;
    private double movementKt;
    private double confidence;
}
