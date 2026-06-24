package org.tash.extensions.agentic;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentEvidenceReceipt {
    private String id;
    private String sourceFamily;
    private String sourceId;
    private String label;
    private String route;
    private String receiptHash;
}
