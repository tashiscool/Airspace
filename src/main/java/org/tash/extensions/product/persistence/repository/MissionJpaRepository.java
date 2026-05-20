package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.MissionEntity;

import java.util.List;
import java.util.Optional;
import java.util.UUID;

@Dependent
public class MissionJpaRepository extends JpaRepository<MissionEntity, UUID> {
    @Inject
    public MissionJpaRepository(EntityManager entityManager) {
        super(entityManager, MissionEntity.class);
    }

    public Optional<MissionEntity> findByMissionNumber(String missionNumber) {
        List<MissionEntity> results = entityManager.createQuery(
                        "select m from MissionEntity m where m.missionNumber = :missionNumber", MissionEntity.class)
                .setParameter("missionNumber", missionNumber)
                .setMaxResults(1)
                .getResultList();
        return results.stream().findFirst();
    }
}
