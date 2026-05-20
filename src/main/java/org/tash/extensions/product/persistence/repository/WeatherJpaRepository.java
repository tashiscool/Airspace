package org.tash.extensions.product.persistence.repository;

import jakarta.enterprise.context.Dependent;
import jakarta.inject.Inject;
import jakarta.persistence.EntityManager;
import org.tash.extensions.product.persistence.entity.WeatherProductEntity;

import java.time.ZonedDateTime;
import java.util.List;

@Dependent
public class WeatherJpaRepository extends JpaRepository<WeatherProductEntity, String> {
    @Inject
    public WeatherJpaRepository(EntityManager entityManager) {
        super(entityManager, WeatherProductEntity.class);
    }

    public List<WeatherProductEntity> findValidAt(ZonedDateTime time) {
        return entityManager.createQuery(
                        "select w from WeatherProductEntity w where w.validStart <= :time and w.validEnd >= :time",
                        WeatherProductEntity.class)
                .setParameter("time", time)
                .getResultList();
    }
}
