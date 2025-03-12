package org.tash.core.uncertainty;

public class FuzzyMembershipFunction {
    public double membership(double x) {
        return 0.0;
    }
    public double getMembership(double x) {
        return membership(x);
    }
    public double getMembership() {
        return 0.0;
    }
    public double getMembership(double x, double y) {
        return membership(x);
    }
}
