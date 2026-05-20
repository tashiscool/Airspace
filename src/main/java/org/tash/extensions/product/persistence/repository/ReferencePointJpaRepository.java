package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.ReferencePointEntity;

import java.util.List;
import java.util.UUID;

@Dependent
public class ReferencePointJpaRepository extends JpaRepository<ReferencePointEntity, UUID> {
    @Inject
    public ReferencePointJpaRepository(EntityManager entityManager) {
        super(entityManager, ReferencePointEntity.class);
    }

    public List<ReferencePointEntity> findByType(String pointType) {
        return entityManager.createQuery(
                        "select r from ReferencePointEntity r where upper(r.pointType) = upper(:pointType) order by r.identifier",
                        ReferencePointEntity.class)
                .setParameter("pointType", pointType)
                .getResultList();
    }
}
