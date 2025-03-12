package org.tash.core.uncertainty;

import lombok.*;
import lombok.experimental.SuperBuilder;

import java.util.HashMap;
import java.util.Map;

@SuperBuilder
@Builder
@Data
@NoArgsConstructor
@AllArgsConstructor
public class FuzzyConjunction {
    private Map<String, Double> membershipFunctions = new HashMap<>();

    public FuzzyConjunction membershipFunction(String name, double value) {
        membershipFunctions.put(name, value);
        return this;
    }
}
