package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.ApprovalEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class ApprovalJpaRepository extends JpaRepository<ApprovalEntity, UUID> {
    @Inject
    public ApprovalJpaRepository(EntityManager entityManager) {
        super(entityManager, ApprovalEntity.class);
    }

    public List<ApprovalEntity> findByReservationId(UUID reservationId) {
        return entityManager.createQuery(
                        "select a from ApprovalEntity a where a.reservationId = :reservationId order by a.updatedAt",
                        ApprovalEntity.class)
                .setParameter("reservationId", reservationId)
                .getResultList();
    }
}
