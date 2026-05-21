import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate, useParams } from 'react-router-dom';
import { Layers, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice } from '../components/Notices';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';
import { conflictReviewSummary } from '../lib/deconflictionView';
import {
  readAcceptedConflictReviews,
  writeAcceptedConflictReviews,
  writeWorkbenchJson,
  type WorkbenchSelection
} from '../lib/workbenchState';
import type { AirspaceFeature } from '../types';

export function DeconflictionPage() {
  const { reservationId = '' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [severityFilter, setSeverityFilter] = useState('ALL');
  const [typeFilter, setTypeFilter] = useState('ALL');
  const [selectedConflictId, setSelectedConflictId] = useState<string | undefined>();
  const [acceptedConflicts, setAcceptedConflicts] = useState<Set<string>>(new Set());
  const [forceReason, setForceReason] = useState('Operational override from deconfliction workspace');
  const [consoleLines, setConsoleLines] = useState<Array<{ level: string; text: string }>>([
    { level: 'INFO', text: 'Deconfliction workspace ready.' }
  ]);
  const missions = useQuery({ queryKey: ['missions'], queryFn: api.missions, enabled: !reservationId });
  const details = useQuery({
    queryKey: ['mission', missions.data?.[0]?.id, 'deconfliction'],
    queryFn: () => api.mission(missions.data![0].id),
    enabled: !reservationId && !!missions.data?.[0]?.id
  });
  const scopedReservationId = reservationId || details.data?.reservations.find((reservation) => reservation.conflictCount > 0)?.id || details.data?.reservations[0]?.id || '';
  const features = useQuery({
    queryKey: ['reservation-features', scopedReservationId],
    queryFn: () => api.reservationFeatures(scopedReservationId),
    enabled: !!scopedReservationId
  });
  const forceDeconflict = useMutation({
    mutationFn: () => api.forceDeconflictReservation(scopedReservationId, 'planner', forceReason),
    onSuccess: () => {
      pushConsole('WARN', `FORCE DECONFLICT submitted for ${scopedReservationId}: ${forceReason}`);
      queryClient.invalidateQueries({ queryKey: ['reservation-features', scopedReservationId] });
    }
  });
  const conflicts = (features.data?.features ?? []).filter((feature) => {
    const props = feature.properties ?? {};
    return props.featureKind === 'conflict' || props.constraintType === 'CARF_CONFLICT';
  });
  const filteredConflicts = useMemo(() => conflicts.filter((feature) => {
    const props = feature.properties ?? {};
    const severity = conflictSeverity(feature);
    const type = String(props.conflictType ?? props.constraintType ?? props.featureKind ?? 'CONFLICT').toUpperCase();
    return (severityFilter === 'ALL' || severity === severityFilter) && (typeFilter === 'ALL' || type.includes(typeFilter));
  }), [conflicts, severityFilter, typeFilter]);
  const selectedConflict = filteredConflicts.find((feature) => String(feature.id) === selectedConflictId) ?? filteredConflicts[0];
  const selectedSummary = useMemo(() => conflictReviewSummary(selectedConflict), [selectedConflict]);
  const selectedAccepted = selectedConflict ? acceptedConflicts.has(String(selectedConflict.id)) : false;
  useEffect(() => {
    if (!scopedReservationId) return;
    setAcceptedConflicts(readAcceptedConflictReviews(scopedReservationId));
  }, [scopedReservationId]);
  useEffect(() => {
    if (!scopedReservationId) return;
    writeAcceptedConflictReviews(scopedReservationId, acceptedConflicts);
  }, [scopedReservationId, acceptedConflicts]);
  useEffect(() => {
    const selection: WorkbenchSelection = {
      reservationId: scopedReservationId || undefined,
      sourceFamily: 'CARF_ALTRV',
      label: scopedReservationId ? `Deconfliction ${scopedReservationId}` : 'Deconfliction Review',
      conflictCount: conflicts.length,
      lockState: selectedConflict ? conflictSeverity(selectedConflict) : 'CLEAR'
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [scopedReservationId, conflicts.length, selectedConflict]);
  const conflictTypes = useMemo(() => {
    const values = new Set(conflicts.map((feature) => String(feature.properties?.conflictType ?? feature.properties?.constraintType ?? 'CONFLICT').toUpperCase()));
    return ['ALL', ...values];
  }, [conflicts]);
  const conflictColumn = createColumnHelper<AirspaceFeature>();
  const columns = [
    conflictColumn.accessor((row) => String(row.properties?.firstReservationId ?? row.properties?.first ?? ''), { id: 'first', header: 'First' }),
    conflictColumn.accessor((row) => String(row.properties?.secondReservationId ?? row.properties?.second ?? ''), { id: 'second', header: 'Second' }),
    conflictColumn.accessor((row) => String(row.properties?.verticalSeparationFeet ?? row.properties?.vertical ?? '-'), { id: 'vertical', header: 'Vertical' }),
    conflictColumn.accessor((row) => String(row.properties?.minimumLateralDistanceNauticalMiles ?? row.properties?.lateral ?? '-'), { id: 'lateral', header: 'Lateral' }),
    conflictColumn.accessor((row) => conflictSeverity(row), { id: 'severity', header: 'Severity', cell: (info) => <StatusBadge value={info.getValue()} /> }),
    conflictColumn.accessor((row) => acceptedConflicts.has(String(row.id)) ? 'ACCEPTED' : 'OPEN', { id: 'review', header: 'Review', cell: (info) => <StatusBadge value={info.getValue()} /> })
  ];

  function pushConsole(level: string, text: string) {
    setConsoleLines((current) => [{ level, text }, ...current].slice(0, 80));
  }

  function runPass() {
    pushConsole('INFO', `Review pass over ${filteredConflicts.length}/${conflicts.length} conflict feature(s).`);
    filteredConflicts.slice(0, 8).forEach((feature) => {
      pushConsole(conflictSeverity(feature) === 'HIGH' ? 'ERR' : 'WARN', `${String(feature.id ?? 'conflict')} ${String(feature.properties?.explanation ?? feature.properties?.rationale ?? 'requires review')}`);
    });
    if (!filteredConflicts.length) pushConsole('INFO', 'No conflicts match current filters.');
  }

  function toggleSelectedConflictReview() {
    if (!selectedConflict) return;
    const id = String(selectedConflict.id);
    setAcceptedConflicts((current) => {
      const next = new Set(current);
      if (next.has(id)) {
        next.delete(id);
        pushConsole('WARN', `Reopened conflict review for ${id}.`);
      } else {
        next.add(id);
        pushConsole('INFO', `Accepted conflict review for ${id}.`);
      }
      return next;
    });
  }

  return (
    <section className="deconfliction-workspace">
      <div className="page-header">
        <div>
          <h2>Deconfliction Review</h2>
          <p>{scopedReservationId ? `Reservation ${scopedReservationId}` : 'Open a reservation to review conflict overlays.'}</p>
        </div>
        <StatusBadge value={conflicts.length ? 'CONFLICTS' : 'CLEAR'} />
      </div>
      <div className="toolbar wrap workbench-toolbar">
        <button disabled={!scopedReservationId} onClick={runPass}><Layers size={14} /> Review Pass</button>
        <button className="secondary" disabled={!selectedConflict} onClick={toggleSelectedConflictReview}>{selectedAccepted ? 'Reopen Selected' : 'Accept Selected'}</button>
        <button className="secondary" disabled={!scopedReservationId} onClick={() => forceDeconflict.mutate()}><ShieldAlert size={14} /> Force Deconflict</button>
        <label className="inline-field">Reason<input value={forceReason} onChange={(event) => setForceReason(event.target.value)} /></label>
        {scopedReservationId && <button className="secondary" onClick={() => navigate(`/missions/${details.data?.mission.id ?? 'unknown'}/reservations/${scopedReservationId}`)}>Open Reservation</button>}
      </div>
      <div className="filter-strip">
        {['ALL', 'LOW', 'MEDIUM', 'HIGH'].map((severity) => (
          <button key={severity} className={severityFilter === severity ? 'chip active' : 'chip'} onClick={() => setSeverityFilter(severity)}>{severity}</button>
        ))}
        <span className="filter-divider" />
        {conflictTypes.map((type) => (
          <button key={type} className={typeFilter === type ? 'chip active' : 'chip'} onClick={() => setTypeFilter(type)}>{type}</button>
        ))}
      </div>
      <div className="notice-stack">
        <ErrorNotice error={missions.error} title="Missions unavailable" />
        <ErrorNotice error={details.error} title="Mission detail unavailable" />
        <ErrorNotice error={features.error} title="Deconfliction features unavailable" />
        <MutationNotice mutation={forceDeconflict} label="Force deconflict" />
      </div>
      <div className="workspace-grid-main">
        <section className="panel">
          <div className="panel-heading"><h3>Conflict Table</h3><span>{filteredConflicts.length}/{conflicts.length}</span></div>
          <DataTable
            data={filteredConflicts}
            columns={columns}
            onRowClick={(feature) => setSelectedConflictId(String(feature.id))}
            onRowDoubleClick={(feature) => {
              setSelectedConflictId(String(feature.id));
              pushConsole('INFO', `Focused map/detail on ${String(feature.id ?? 'selected conflict')}.`);
            }}
            isRowSelected={(feature) => String(feature.id) === String(selectedConflict?.id)}
          />
          <div className="conflict-detail-grid">
            <div className="vertical-summary">
              <strong>Selected Conflict</strong>
              <p>{selectedSummary?.explanation ?? 'No selected conflict.'}</p>
              {selectedSummary && (
                <div className="conflict-metrics" aria-label="Conflict vertical lateral time summary">
                  {selectedSummary.metrics.map((metric) => (
                    <div className={`conflict-metric ${metric.status.toLowerCase()}`} key={metric.label}>
                      <div>
                        <span>{metric.label}</span>
                        <strong>
                          {metric.value == null ? '—' : `${metric.value} ${metric.unit}`}
                          {metric.threshold != null ? ` / ${metric.threshold}` : ''}
                        </strong>
                      </div>
                      <div className="metric-bar">
                        <i style={{ width: `${metric.percent}%` }} />
                      </div>
                    </div>
                  ))}
                </div>
              )}
              {selectedSummary && (selectedSummary.sourceRatios.length > 0 || selectedSummary.sourcePoints.length > 0) && (
                <div className="source-metadata">
                  {selectedSummary.sourceRatios.length > 0 && <span>Ratios: {selectedSummary.sourceRatios.join(', ')}</span>}
                  {selectedSummary.sourcePoints.length > 0 && <span>Points: {selectedSummary.sourcePoints.join(', ')}</span>}
                </div>
              )}
            </div>
            <dl className="mini-kv conflict-kv">
              <dt>Severity</dt><dd>{selectedSummary?.severity ?? '-'}</dd>
              <dt>Vertical</dt><dd>{String(selectedConflict?.properties?.verticalSeparationFeet ?? selectedConflict?.properties?.vertical ?? '-')}</dd>
              <dt>Lateral NM</dt><dd>{String(selectedConflict?.properties?.minimumLateralDistanceNauticalMiles ?? selectedConflict?.properties?.lateral ?? '-')}</dd>
              <dt>Duration</dt><dd>{String(selectedConflict?.properties?.durationMinutes ?? selectedConflict?.properties?.belowMinimumDuration ?? '-')}</dd>
              <dt>Review</dt><dd>{selectedAccepted ? 'Accepted' : 'Open'}</dd>
            </dl>
          </div>
          <div className="console-panel">
            {consoleLines.map((line, index) => (
              <div key={`${line.text}:${index}`} className={`console-line ${line.level.toLowerCase()}`}><span>{line.level}</span>{line.text}</div>
            ))}
          </div>
        </section>
        <OperationsMap
          features={features.data}
          selectedFeatureId={selectedConflict ? String(selectedConflict.id) : undefined}
          onSelectedFeatureIdChange={setSelectedConflictId}
        />
      </div>
    </section>
  );
}

function conflictSeverity(feature: AirspaceFeature) {
  const props = feature.properties ?? {};
  const explicit = String(props.severity ?? '').toUpperCase();
  if (explicit) return explicit;
  if (props.belowMinimumDuration === true) return 'LOW';
  if (Number(props.minimumLateralDistanceNauticalMiles ?? 999) < 60) return 'HIGH';
  if (Number(props.verticalSeparationFeet ?? 99999) < 1000) return 'HIGH';
  return 'MEDIUM';
}
