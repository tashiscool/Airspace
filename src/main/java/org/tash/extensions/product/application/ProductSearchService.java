package org.tash.extensions.product.application;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.List;
import java.util.stream.Collectors;

@ApplicationScoped
public class ProductSearchService {
    private final AirspaceProductService productService;

    @Inject
    public ProductSearchService(AirspaceProductService productService) {
        this.productService = productService;
    }

    public List<ProductDtos.SearchResultSummary> searchMissions(String query) {
        return byType(query, "mission");
    }

    public List<ProductDtos.SearchResultSummary> searchMessages(String query) {
        return byType(query, "message");
    }

    public List<ProductDtos.SearchResultSummary> searchDecisions(String query) {
        return byType(query, "decision");
    }

    public List<ProductDtos.SearchResultSummary> searchHistory(String query) {
        return byType(query, "history");
    }

    private List<ProductDtos.SearchResultSummary> byType(String query, String type) {
        return productService.search(query).stream()
                .filter(result -> type.equals(result.getType()))
                .collect(Collectors.toList());
    }
}
