package org.tash.extensions.weather.pirep;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ListPirepRepositoryView implements PirepRepositoryView {
    private final List<PirepReport> reports;

    public ListPirepRepositoryView(List<PirepReport> reports) {
        this.reports = reports == null ? new ArrayList<>() : new ArrayList<>(reports);
    }

    @Override
    public List<PirepReport> recentReports() {
        return Collections.unmodifiableList(reports);
    }
}
