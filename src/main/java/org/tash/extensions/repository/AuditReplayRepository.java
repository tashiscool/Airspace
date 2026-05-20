package org.tash.extensions.repository;

import org.tash.extensions.engine.OperationalDecisionAuditEnvelope;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;

import java.util.List;
import java.util.Optional;

public interface AuditReplayRepository {
    String saveAuditEnvelope(OperationalDecisionAuditEnvelope envelope);

    String saveReplayBundle(OperationalDecisionReplayBundle bundle);

    Optional<String> findJsonById(String id);

    List<String> listIds();
}
