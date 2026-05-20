package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.MessageEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class MessageJpaRepository extends JpaRepository<MessageEntity, UUID> {
    @Inject
    public MessageJpaRepository(EntityManager entityManager) {
        super(entityManager, MessageEntity.class);
    }

    public List<MessageEntity> findByMissionId(UUID missionId) {
        return entityManager.createQuery("select m from MessageEntity m where m.missionId = :missionId", MessageEntity.class)
                .setParameter("missionId", missionId)
                .getResultList();
    }
}
