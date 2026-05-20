package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.NotamEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class NotamJpaRepository extends JpaRepository<NotamEntity, UUID> {
    @Inject
    public NotamJpaRepository(EntityManager entityManager) {
        super(entityManager, NotamEntity.class);
    }

    public List<NotamEntity> findByReservationId(UUID reservationId) {
        return entityManager.createQuery(
                        "select n from NotamEntity n where n.reservationId = :reservationId order by n.updatedAt",
                        NotamEntity.class)
                .setParameter("reservationId", reservationId)
                .getResultList();
    }
}
