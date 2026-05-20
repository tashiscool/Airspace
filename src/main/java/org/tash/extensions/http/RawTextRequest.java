package org.tash.extensions.http;

import lombok.Data;

@Data
public class RawTextRequest {
    private String rawText;
    private String actor;
    private String note;
}
