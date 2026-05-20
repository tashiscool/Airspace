package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.HistoryEventEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class HistoryEventJpaRepository extends JpaRepository<HistoryEventEntity, UUID> {
    @Inject
    public HistoryEventJpaRepository(EntityManager entityManager) {
        super(entityManager, HistoryEventEntity.class);
    }

    public List<HistoryEventEntity> findByAggregate(String aggregateType, String aggregateId) {
        return entityManager.createQuery(
                        "select h from HistoryEventEntity h where h.aggregateType = :type and h.aggregateId = :id order by h.createdAt",
                        HistoryEventEntity.class)
                .setParameter("type", aggregateType)
                .setParameter("id", aggregateId)
                .getResultList();
    }
}
