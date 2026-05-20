package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.FeedArtifactEntity;

import java.util.UUID;

@Dependent
public class FeedArtifactJpaRepository extends JpaRepository<FeedArtifactEntity, UUID> {
    @Inject
    public FeedArtifactJpaRepository(EntityManager entityManager) {
        super(entityManager, FeedArtifactEntity.class);
    }
}
