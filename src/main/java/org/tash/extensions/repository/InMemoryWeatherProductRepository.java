package org.tash.extensions.repository;

import org.tash.extensions.engine.CanonicalJson;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class InMemoryWeatherProductRepository implements WeatherProductRepository {
    private final Map<String, WeatherProduct> records = new LinkedHashMap<>();
    private final Map<String, String> json = new LinkedHashMap<>();

    @Override
    public synchronized String save(WeatherProduct product) {
        String id = product == null || product.getId() == null
                ? "weather:" + CanonicalJson.sha256(product)
                : "weather:" + product.getId();
        records.put(id, product);
        json.put(id, CanonicalJson.write(product));
        return id;
    }

    @Override
    public synchronized Optional<WeatherProduct> findById(String id) {
        return Optional.ofNullable(records.get(id));
    }

    @Override
    public synchronized Optional<String> findJsonById(String id) {
        return Optional.ofNullable(json.get(id));
    }

    @Override
    public synchronized List<String> listIds() {
        return new ArrayList<>(records.keySet());
    }
}
