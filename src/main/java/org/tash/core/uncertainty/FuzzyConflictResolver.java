package org.tash.core.uncertainty;

import org.tash.data.GeoCoordinate;
import org.tash.event.conflict.SeparationConflict;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.trajectory.TrajectorySegment;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Fuzzy logic system for conflict resolution with imprecise data
 */
public class FuzzyConflictResolver {
    
    // Minimum horizontal separation for conflict (nautical miles)
    private final double minHorizontalSeparation;
    
    // Minimum vertical separation for conflict (feet)
    private final double minVerticalSeparation;
    
    // Fuzzy membership functions
    private final Map<String, FuzzyMembershipFunction> membershipFunctions;
    
    // Fuzzy rule base
    private final List<FuzzyRule> rules;
    
    /**
     * Constructor with default separation minima
     */
    public FuzzyConflictResolver() {
        this(5.0, 1000.0);
    }
    
    /**
     * Constructor with custom separation minima
     * 
     * @param minHorizontalSeparation Minimum horizontal separation in nautical miles
     * @param minVerticalSeparation Minimum vertical separation in feet
     */
    public FuzzyConflictResolver(double minHorizontalSeparation, double minVerticalSeparation) {
        this.minHorizontalSeparation = minHorizontalSeparation;
        this.minVerticalSeparation = minVerticalSeparation;
        
        // Initialize membership functions
        this.membershipFunctions = new HashMap<>();
        initializeMembershipFunctions();
        
        // Initialize rule base
        this.rules = new ArrayList<>();
        initializeRuleBase();
    }

    private void initializeRuleBase() {
        // Define fuzzy rules
        rules.add(FuzzyRule.builder()
               .antecedent(FuzzyConjunction.builder().build()
                       .membershipFunction("horizontalSeparation.veryClose", 1.0)
                       .membershipFunction("verticalSeparation.veryClose", 1.0))
               .consequent(SeparationConflict.builder()
                       .horizontalSeparation(minHorizontalSeparation)
                       .verticalSeparation(minVerticalSeparation)
                       .build())
               .build());
    }

    /**
     * Initialize fuzzy membership functions
     */
    private void initializeMembershipFunctions() {
        // Horizontal separation membership functions
        membershipFunctions.put("horizontalSeparation.veryClose", new TrapezoidMembershipFunction(0, 0, 2, 3));
        membershipFunctions.put("horizontalSeparation.close", new TrapezoidMembershipFunction(2, 3, 4, 5));
        membershipFunctions.put("horizontalSeparation.adequate", new TrapezoidMembershipFunction(4, 5, 7, 8));
        membershipFunctions.put("horizontalSeparation.far", new TrapezoidMembershipFunction(7, 8, 12, 15));
        membershipFunctions.put("horizontalSeparation.veryFar", new TrapezoidMembershipFunction(12, 15, 30, 30));
        
        // Vertical separation membership functions
        membershipFunctions.put("verticalSeparation.veryClose", new TrapezoidMembershipFunction(0, 0, 500, 700));
        membershipFunctions.put("verticalSeparation.close", new TrapezoidMembershipFunction(500, 700, 900, 1000));
        membershipFunctions.put("verticalSeparation.adequate", new TrapezoidMembershipFunction(900, 1000, 1500, 2000));
        membershipFunctions.put("verticalSeparation.far", new TrapezoidMembershipFunction(1500, 2000, 3000, 4000));
        membershipFunctions.put("verticalSeparation.veryFar", new TrapezoidMembershipFunction(3000, 4000, 10000, 10000));
        
        // Closure rate membership functions (knots)
        membershipFunctions.put("closureRate.converging", new TrapezoidMembershipFunction(-1000, -1000, -50, -10));
        membershipFunctions.put("closureRate.stable", new TrapezoidMembershipFunction(-50, -10, 10, 50));
        membershipFunctions.put("closureRate.diverging", new TrapezoidMembershipFunction(10, 50, 1000, 1000));
        
        // Position uncertainty membership functions
        membershipFunctions.put("positionUncertainty.low", new TrapezoidMembershipFunction(0, 0, 0.2, 0.5));
        membershipFunctions.put("positionUncertainty.medium", new TrapezoidMembershipFunction(0.2, 0.5, 1.0, 1.5));
        membershipFunctions.put("positionUncertainty.high", new TrapezoidMembershipFunction(1.0, 1.5, 5, 5));
        
        // Conflict risk membership functions
        membershipFunctions.put("conflictRisk.low", new TrapezoidMembershipFunction(0, 0, 0.3, 0.4));
        membershipFunctions.put("conflictRisk.medium", new TrapezoidMembershipFunction(0.3, 0.4, 0.6, 0.7));
        membershipFunctions.put("conflictRisk.high", new TrapezoidMembershipFunction(0.6, 0.7, 1.0, 1.0));
        
        // Resolution actions membership functions (heading change in degrees)
        membershipFunctions.put("headingChange.none", new TrapezoidMembershipFunction(0, 0, 0, 5));
        membershipFunctions.put("headingChange.slight", new TrapezoidMembershipFunction(0, 5, 10, 15));
        membershipFunctions.put("headingChange.moderate", new TrapezoidMembershipFunction(10, 15, 25, 30));
        membershipFunctions.put("headingChange.significant", new TrapezoidMembershipFunction(25, 30, 45, 60));
        membershipFunctions.put("headingChange.extreme", new TrapezoidMembershipFunction(45, 60, 90, 90));
        
        // Resolution actions membership functions (altitude change in feet)
        membershipFunctions.put("altitudeChange.none", new TrapezoidMembershipFunction(0, 0, 0, 500));
        membershipFunctions.put("altitudeChange.slight", new TrapezoidMembershipFunction(0, 500, 1000, 1500));
        membershipFunctions.put("altitudeChange.moderate", new TrapezoidMembershipFunction(1000, 1500, 2000, 2500));
        membershipFunctions.put("altitudeChange.significant", new TrapezoidMembershipFunction(2000, 2500, 3000, 4000));
        membershipFunctions.put("altitudeChange.extreme", new TrapezoidMembershipFunction(3000, 4000, 5000, 5000));
        
        // Resolution actions membership functions (speed change in knots)
        membershipFunctions.put("speedChange.none", new TrapezoidMembershipFunction(0, 0, 0, 50));
        membershipFunctions.put("speedChange.slight", new TrapezoidMembershipFunction(0, 50, 100, 150));
        membershipFunctions.put("speedChange.moderate", new TrapezoidMembershipFunction(100, 150, 200, 250));
        membershipFunctions.put("speedChange.significant", new TrapezoidMembershipFunction(200, 250, 300, 350));
        membershipFunctions.put("speedChange.extreme", new TrapezoidMembershipFunction(300, 350, 450, 450));
    }

    /**
     * Resolve conflicts between two trajectory segments
     *
     * @param segment1 First trajectory segment
     * @param segment2 Second trajectory segment
     * @return Resolution action
     */
    public ResolutionAction resolveConflicts(TrajectorySegment segment1, TrajectorySegment segment2) {
        if (segment1 == null || segment2 == null) {
            return ResolutionAction.builder()
                    .riskScore(0)
                    .rationale("No resolution required: one or both trajectory segments are missing")
                    .build();
        }

        double horizontalSeparation = horizontalSeparation(segment1, segment2);
        double verticalSeparation = verticalSeparation(segment1, segment2);
        double horizontalRisk = 1.0 - membership("horizontalSeparation.adequate", horizontalSeparation);
        double verticalRisk = 1.0 - membership("verticalSeparation.adequate", verticalSeparation);
        double veryCloseBoost = Math.max(membership("horizontalSeparation.veryClose", horizontalSeparation),
                membership("verticalSeparation.veryClose", verticalSeparation));
        double risk = clamp((horizontalRisk * 0.55) + (verticalRisk * 0.35) + (veryCloseBoost * 0.10));

        if (horizontalSeparation >= minHorizontalSeparation && verticalSeparation >= minVerticalSeparation) {
            return ResolutionAction.builder()
                    .riskScore(Math.min(risk, 0.25))
                    .rationale("No maneuver required: horizontal and vertical minima are satisfied")
                    .build();
        }

        double headingChange = risk < 0.35 ? 10 : risk < 0.70 ? 25 : 45;
        double altitudeChange = verticalSeparation < minVerticalSeparation
                ? Math.max(500, minVerticalSeparation - verticalSeparation)
                : 0;
        double speedChange = horizontalSeparation < minHorizontalSeparation ? Math.max(25, risk * 150) : 0;

        return ResolutionAction.builder()
                .headingChangeDegrees(headingChange)
                .altitudeChangeFeet(altitudeChange)
                .speedChangeKnots(speedChange)
                .riskScore(risk)
                .rationale("Fuzzy resolution based on horizontal=" + horizontalSeparation
                        + "NM vertical=" + verticalSeparation + "ft")
                .build();
    }

    private double horizontalSeparation(TrajectorySegment first, TrajectorySegment second) {
        GeoCoordinate a = first.getSource().getCoordinate();
        GeoCoordinate b = second.getSource().getCoordinate();
        return GeodeticCalculator.vincentyDistance(a, b);
    }

    private double verticalSeparation(TrajectorySegment first, TrajectorySegment second) {
        return Math.abs(first.getSource().getCoordinate().getAltitude()
                - second.getSource().getCoordinate().getAltitude());
    }

    private double membership(String name, double value) {
        FuzzyMembershipFunction function = membershipFunctions.get(name);
        return function == null ? 0 : function.getMembership(value);
    }

    private double clamp(double value) {
        return Math.max(0, Math.min(1, value));
    }
}
