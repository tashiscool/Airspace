package org.tash.extensions.http;

import lombok.Data;

@Data
public class ConflictReviewRequest {
    private boolean accepted;
    private String reviewer;
    private String note;
}
