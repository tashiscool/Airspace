package org.tash.extensions.repository;

import org.tash.extensions.weather.pirep.PirepIngestResult;

import java.util.List;
import java.util.Optional;

public interface PirepReportRepository {
    String save(PirepIngestResult result);

    Optional<PirepIngestResult> findById(String id);

    Optional<String> findJsonById(String id);

    List<String> listIds();
}
