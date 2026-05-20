package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.ApreqEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class ApreqJpaRepository extends JpaRepository<ApreqEntity, UUID> {
    @Inject
    public ApreqJpaRepository(EntityManager entityManager) {
        super(entityManager, ApreqEntity.class);
    }

    public List<ApreqEntity> findByReservationId(UUID reservationId) {
        return entityManager.createQuery(
                        "select a from ApreqEntity a where a.reservationId = :reservationId order by a.updatedAt",
                        ApreqEntity.class)
                .setParameter("reservationId", reservationId)
                .getResultList();
    }
}
