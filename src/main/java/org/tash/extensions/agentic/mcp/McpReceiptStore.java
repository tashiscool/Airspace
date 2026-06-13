package org.tash.extensions.agentic.mcp;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

@ApplicationScoped
public class McpReceiptStore {
    private final Map<String, McpEvidenceReceipt> receipts = Collections.synchronizedMap(new LinkedHashMap<>());

    public void save(McpEvidenceReceipt receipt) {
        if (receipt != null && receipt.getId() != null) {
            receipts.put(receipt.getId(), receipt);
        }
    }

    public Optional<McpEvidenceReceipt> find(String id) {
        return Optional.ofNullable(id == null ? null : receipts.get(id));
    }

    public List<McpEvidenceReceipt> list(Integer limit) {
        int max = limit == null || limit <= 0 ? 25 : Math.min(limit, 250);
        List<McpEvidenceReceipt> values;
        synchronized (receipts) {
            values = new ArrayList<>(receipts.values());
        }
        Collections.reverse(values);
        return values.size() > max ? new ArrayList<>(values.subList(0, max)) : values;
    }
}
