package org.tash.extensions.messaging;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class LegacySampleFixture {
    private String name;
    private String text;
}
