package org.tash.core.uncertainty;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.event.conflict.SeparationConflict;

@SuperBuilder
@Builder
@Data
@NoArgsConstructor
@AllArgsConstructor
public class FuzzyRule {
    private FuzzyConjunction antecedent;
    private SeparationConflict consequent;
    private FuzzyConjunction conjunction;
    private double weight;
}
