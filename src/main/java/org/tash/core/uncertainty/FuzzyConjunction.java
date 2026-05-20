package org.tash.core.uncertainty;

import lombok.*;

import java.util.HashMap;
import java.util.Map;

@Builder
@Data
@NoArgsConstructor
@AllArgsConstructor
public class FuzzyConjunction {
    @Builder.Default
    private Map<String, Double> membershipFunctions = new HashMap<>();

    public FuzzyConjunction membershipFunction(String name, double value) {
        if (membershipFunctions == null) {
            membershipFunctions = new HashMap<>();
        }
        membershipFunctions.put(name, value);
        return this;
    }
}
