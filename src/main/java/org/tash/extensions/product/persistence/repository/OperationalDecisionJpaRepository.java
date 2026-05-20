package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.OperationalDecisionEntity;

import java.util.UUID;

@Dependent
public class OperationalDecisionJpaRepository extends JpaRepository<OperationalDecisionEntity, UUID> {
    @Inject
    public OperationalDecisionJpaRepository(EntityManager entityManager) {
        super(entityManager, OperationalDecisionEntity.class);
    }
}
