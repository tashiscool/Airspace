package org.tash.extensions.notam.access;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class NotamAccessResult {
    private boolean allowed;
    private List<String> warnings;
    private List<String> errors;
}
