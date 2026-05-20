package org.tash.extensions.repository;

import org.tash.extensions.weather.product.WeatherProduct;

import java.util.List;
import java.util.Optional;

public interface WeatherProductRepository {
    String save(WeatherProduct product);

    Optional<WeatherProduct> findById(String id);

    Optional<String> findJsonById(String id);

    List<String> listIds();
}
