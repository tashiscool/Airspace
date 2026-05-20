package org.tash.extensions.repository;

import org.tash.extensions.engine.OperationalDecisionResult;

import java.util.List;
import java.util.Optional;

public interface OperationalDecisionRepository {
    String save(OperationalDecisionResult result);

    Optional<OperationalDecisionResult> findById(String id);

    Optional<String> findJsonById(String id);

    List<String> listIds();
}
