package org.tash.extensions.repository;

import org.tash.extensions.weather.product.WeatherProduct;

import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class JsonFileWeatherProductRepository implements WeatherProductRepository {
    private final JsonArtifactStore store;
    private final Map<String, WeatherProduct> currentProcessRecords = new LinkedHashMap<>();

    public JsonFileWeatherProductRepository(Path path) {
        this.store = new JsonArtifactStore(path);
    }

    @Override
    public synchronized String save(WeatherProduct product) {
        String id = store.save("weather", product);
        currentProcessRecords.put(id, product);
        return id;
    }

    @Override
    public synchronized Optional<WeatherProduct> findById(String id) {
        return Optional.ofNullable(currentProcessRecords.get(id));
    }

    @Override
    public Optional<String> findJsonById(String id) {
        return store.findJsonById(id);
    }

    @Override
    public List<String> listIds() {
        return store.listIds();
    }
}
