package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.ReservationEntity;

import java.time.ZonedDateTime;
import java.util.List;
import java.util.UUID;

@Dependent
public class ReservationJpaRepository extends JpaRepository<ReservationEntity, UUID> {
    @Inject
    public ReservationJpaRepository(EntityManager entityManager) {
        super(entityManager, ReservationEntity.class);
    }

    public List<ReservationEntity> findForDeconfliction(ZonedDateTime start, ZonedDateTime end) {
        return entityManager.createQuery(
                        "select r from ReservationEntity r where r.effectiveStart <= :end and r.effectiveEnd >= :start",
                        ReservationEntity.class)
                .setParameter("start", start)
                .setParameter("end", end)
                .getResultList();
    }

    public List<ReservationEntity> findByMissionId(UUID missionId) {
        return entityManager.createQuery(
                        "select r from ReservationEntity r where r.mission.id = :missionId order by r.createdAt",
                        ReservationEntity.class)
                .setParameter("missionId", missionId)
                .getResultList();
    }
}
