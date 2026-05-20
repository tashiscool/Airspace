package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class ReferenceDataSyncResult {
    private final boolean accepted;
    @Builder.Default
    private final List<ReferenceDataImportRecord> records = new ArrayList<>();
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<String> errors = new ArrayList<>();

    public List<ReferenceDataImportRecord> getRecords() {
        return Collections.unmodifiableList(records == null ? Collections.emptyList() : records);
    }

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings == null ? Collections.emptyList() : warnings);
    }

    public List<String> getErrors() {
        return Collections.unmodifiableList(errors == null ? Collections.emptyList() : errors);
    }
}
