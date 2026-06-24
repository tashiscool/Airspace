package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class BehaviorStateMachine {
    private String actor;
    private String currentState;
    private String nextAction;
    private boolean humanApprovalRequired;
    private int communicationDelaySeconds;
    private double acceptanceProbability;
    @Builder.Default
    private List<String> transitionHistory = new ArrayList<>();
}
