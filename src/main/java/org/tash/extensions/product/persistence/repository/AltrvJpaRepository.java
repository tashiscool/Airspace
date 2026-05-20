package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.AltrvMessageEntity;

import java.util.UUID;

@Dependent
public class AltrvJpaRepository extends JpaRepository<AltrvMessageEntity, UUID> {
    @Inject
    public AltrvJpaRepository(EntityManager entityManager) {
        super(entityManager, AltrvMessageEntity.class);
    }
}
