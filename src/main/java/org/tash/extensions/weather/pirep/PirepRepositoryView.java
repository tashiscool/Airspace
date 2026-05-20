package org.tash.extensions.weather.pirep;

import java.util.Collections;
import java.util.List;

public interface PirepRepositoryView {
    default List<PirepReport> recentReports() {
        return Collections.emptyList();
    }
}
