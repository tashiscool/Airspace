package org.tash.core;

/**
     * Interface for elements that can be visited (Visitor pattern)
     */
    public interface Visitable {
        void accept(Visitor visitor);
    }