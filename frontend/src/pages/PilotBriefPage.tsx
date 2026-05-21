import { useEffect } from 'react';
import { useQuery } from '@tanstack/react-query';
import { Link, useParams } from 'react-router-dom';
import { Printer, Radio, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { fmtZ, sourceRefLabel, sourceRefRoute } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

export function PilotBriefPage() {
  const { missionId = '' } = useParams();
  const brief = useQuery({
    queryKey: ['pilot-brief', missionId, 'handoff'],
    queryFn: () => api.pilotBrief(missionId),
    enabled: !!missionId
  });
  const data = brief.data;

  useEffect(() => {
    if (!data) return;
    const selection: WorkbenchSelection = {
      missionId,
      sourceFamily: 'WEATHER',
      label: data.missionNumber,
      lockState: `${data.verdict.action} · ${Math.round(data.verdict.confidence * 100)}%`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [data, missionId]);

  return (
    <section className="pilot-brief-workspace">
      <header className="pilot-brief-header">
        <div>
          <span className="brief-kicker">Read-only pilot handoff</span>
          <h1>{data?.missionNumber ?? 'Mission Brief'}</h1>
          <p>Current weather, PIREP, NOTAM, route-impact, and decision-trace summary for cockpit or release briefing.</p>
        </div>
        <div className="brief-actions">
          {data?.verdict && <StatusBadge value={data.verdict.action} />}
          <button onClick={() => window.print()}><Printer size={14} /> Print</button>
          <Link className="brief-link" to={`/missions/${missionId}`}>Workbench</Link>
        </div>
      </header>
      <QueryNotice query={brief} label="Pilot brief" />
      <main className="pilot-brief-grid">
        <section className="brief-card verdict">
          <span>Verdict</span>
          <strong>{data?.verdict.action ?? 'PENDING'}</strong>
          <p>{data?.verdict.summary ?? 'Generate a mission brief once operational inputs are available.'}</p>
          <small>{Math.round((data?.verdict.confidence ?? 0) * 100)}% confidence · {data?.verdict.sourceCount ?? 0} source(s)</small>
        </section>
        <section className="brief-card">
          <span>Recommended Action</span>
          <strong>{data?.verdict.recommendedAction ?? 'Monitor'}</strong>
          <p>{data?.routeImpact.rationale ?? 'No route-impact decision has been loaded.'}</p>
        </section>
        <section className="brief-card">
          <span>Route Impact</span>
          <strong>{data?.routeImpact.action ?? 'PENDING'}</strong>
          <p>{data?.routeImpact.impactedSegmentCount ?? 0} impacted segment(s), {data?.routeImpact.blockingConstraintCount ?? 0} blocking constraint(s).</p>
          <div className="route-source-grid">
            {(data?.routeImpact.sourceRefs?.length ? data.routeImpact.sourceRefs : ['No explicit source refs']).map((ref) => (
              sourceRefRoute(ref) ? (
                <Link key={ref} to={sourceRefRoute(ref)!} className={`route-source route-source-${sourceRefLabel(ref).family.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`}>
                  <strong>{sourceRefLabel(ref).family}</strong>
                  {sourceRefLabel(ref).id || ref}
                </Link>
              ) : (
                <span key={ref} className={`route-source route-source-${sourceRefLabel(ref).family.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`}>
                  <strong>{sourceRefLabel(ref).family}</strong>
                  {sourceRefLabel(ref).id || ref}
                </span>
              )
            ))}
          </div>
        </section>
        <section className="brief-card">
          <span>What Changed</span>
          <strong>{data?.changes.length ?? 0}</strong>
          <p>Weather, PIREP, or NOTAM deltas since the briefing window.</p>
        </section>
      </main>
      <section className="brief-section">
        <h2><Radio size={16} /> Source Artifacts</h2>
        {!!data?.sourceSummaryLines?.length && (
          <div className="brief-source-summary">
            {data.sourceSummaryLines.map((line) => <span key={line}>{line}</span>)}
          </div>
        )}
        <div className="brief-source-list">
          {(data?.changes.length ? data.changes : data?.verdict.sources ?? []).map((source) => (
            <Link
              key={`${source.family}:${source.id}`}
              to={source.route ?? `/messages/${encodeURIComponent(source.id)}`}
              className="brief-source brief-source-link"
            >
              <StatusBadge value={source.severity ?? source.family} />
              <div>
                <strong>{source.family} · {source.label}</strong>
                <p>{source.rationale}</p>
                <small>{fmtZ(source.observedAt)} {source.stale ? '· stale' : ''}</small>
              </div>
            </Link>
          ))}
          {!data?.changes.length && !data?.verdict.sources.length && <p className="muted">No source artifacts are attached to this brief.</p>}
        </div>
      </section>
      <section className="brief-section">
        <h2><ShieldAlert size={16} /> Trace Summary</h2>
        <p>{data?.decisionTraceSummary ?? 'No decision trace summary is available yet.'}</p>
        <pre className="brief-printable">{data?.printableText ?? ''}</pre>
      </section>
    </section>
  );
}
