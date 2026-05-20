package org.tash.extensions.product.persistence.repository;

import jakarta.persistence.EntityManager;

import java.util.List;
import java.util.Optional;

public class JpaRepository<T, ID> {
    protected final EntityManager entityManager;
    private final Class<T> entityClass;

    public JpaRepository(EntityManager entityManager, Class<T> entityClass) {
        this.entityManager = entityManager;
        this.entityClass = entityClass;
    }

    public T save(T entity) {
        return entityManager.merge(entity);
    }

    public Optional<T> findById(ID id) {
        return Optional.ofNullable(entityManager.find(entityClass, id));
    }

    public List<T> listAll() {
        return entityManager.createQuery("select e from " + entityClass.getSimpleName() + " e", entityClass)
                .getResultList();
    }

    public long count() {
        return entityManager.createQuery("select count(e) from " + entityClass.getSimpleName() + " e", Long.class)
                .getSingleResult();
    }

    public void delete(T entity) {
        entityManager.remove(entityManager.contains(entity) ? entity : entityManager.merge(entity));
    }
}
