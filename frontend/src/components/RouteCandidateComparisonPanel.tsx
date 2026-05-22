import { Link } from 'react-router-dom';
import type { ConstraintImpactSummary, RouteCandidateComparisonSummary, RouteImpactSummary, RerouteTraceSummary } from '../types';
import { sourceRefLabel, sourceRefRoute } from '../lib/viewModels';
import { StatusBadge } from './StatusBadge';

export function RouteCandidateComparisonPanel({
  routeImpact,
  title = 'Route Candidate Comparison',
  selectedCandidateId,
  onCandidateSelect
}: {
  routeImpact?: Partial<RouteImpactSummary>;
  title?: string;
  selectedCandidateId?: string;
  onCandidateSelect?: (candidateId: string) => void;
}) {
  const comparisons = routeImpact?.candidateComparisons ?? [];
  const trace = routeImpact?.whyRerouteTrace ?? [];
  const legacyCandidates = routeImpact?.avoidanceCandidates ?? [];

  return (
    <div className="route-comparison-panel">
      <div className="panel-heading compact">
        <h4>{title}</h4>
        <span>{comparisons.length ? `${comparisons.length} alternate(s)` : 'No structured alternate'}</span>
      </div>
      <div className="route-baseline-grid">
        <Metric label="Original NM" value={formatNumber(routeImpact?.originalRouteDistanceNm)} />
        <Metric label="Minutes" value={formatNumber(routeImpact?.originalRouteEstimatedMinutes)} />
        <Metric label="Fuel lb" value={formatNumber(routeImpact?.originalRouteEstimatedFuelLb)} />
        <Metric label="Cost $" value={formatCurrency(routeImpact?.originalRouteEstimatedCostUsd)} />
      </div>
      {comparisons.map((candidate) => (
        <CandidateCard
          key={candidate.id}
          candidate={candidate}
          selected={candidate.id === selectedCandidateId}
          onSelect={onCandidateSelect}
        />
      ))}
      {!comparisons.length && !!legacyCandidates.length && (
        <div className="source-ref-grid">
          {legacyCandidates.map((item) => <span key={item}>{item}</span>)}
        </div>
      )}
      {!comparisons.length && !legacyCandidates.length && (
        <p className="muted">No alternate corridor has been attached to this route impact yet.</p>
      )}
      {!!trace.length && (
        <div className="reroute-trace-list">
          <strong>Why This Reroute?</strong>
          {trace.slice(0, 8).map((item, index) => <TraceRow key={`${item.ruleId}-${index}`} trace={item} />)}
        </div>
      )}
    </div>
  );
}

function CandidateCard({
  candidate,
  selected,
  onSelect
}: {
  candidate: RouteCandidateComparisonSummary;
  selected?: boolean;
  onSelect?: (candidateId: string) => void;
}) {
  const cost = candidate.cost ?? {};
  return (
    <article className={selected ? 'route-candidate-card selected' : 'route-candidate-card'}>
      <div className="route-candidate-header">
        <div>
          <strong>{candidate.label || candidate.id}</strong>
          <p>{candidate.rationale || 'Deterministic alternate corridor generated around blocking constraints.'}</p>
        </div>
        <div className="route-candidate-actions">
          <StatusBadge value={`${Math.round((candidate.confidence ?? 0) * 100)}% confidence`} />
          {onSelect && <button type="button" className="secondary" onClick={() => onSelect(candidate.id)}>Show On Map</button>}
        </div>
      </div>
      <div className="route-cost-grid">
        <Metric label="Distance" value={`${formatNumber(cost.distanceNm)} NM`} />
        <Metric label="Added" value={`+${formatNumber(cost.additionalDistanceNm)} NM`} />
        <Metric label="Delay" value={`+${formatNumber(cost.additionalMinutes)} min`} />
        <Metric label="Fuel" value={`+${formatNumber(cost.additionalFuelLb)} lb`} />
        <Metric label="Cost" value={`+$${formatCurrency(cost.additionalCostUsd)}`} />
      </div>
      <small className="route-cost-assumptions">
        Cost model: {formatNumber(cost.cruiseSpeedKnots)} kt cruise · {formatNumber(cost.fuelBurnLbPerNm)} lb/NM · ${formatDecimal(cost.fuelCostUsdPerLb, 2)} per lb · ${formatCurrency(cost.delayCostUsdPerMinute)} per delay min
      </small>
      <ConstraintList title="Avoided Hazards" items={candidate.avoidedConstraints ?? []} empty="No avoided hazards listed." />
      <ConstraintList title="Residual Constraints" items={candidate.residualConstraints ?? []} empty="No residual constraints." />
      {!!candidate.sourceRefs?.length && (
        <div className="route-source-grid compact">
          {candidate.sourceRefs.map((ref) => <SourceRefChip key={ref} refValue={ref} />)}
        </div>
      )}
      {!!candidate.trace?.length && (
        <div className="reroute-trace-list compact">
          {candidate.trace.slice(0, 4).map((item, index) => <TraceRow key={`${candidate.id}-${item.ruleId}-${index}`} trace={item} />)}
        </div>
      )}
    </article>
  );
}

function ConstraintList({ title, items, empty }: { title: string; items: ConstraintImpactSummary[]; empty: string }) {
  return (
    <div className="constraint-impact-list">
      <strong>{title}</strong>
      {items.length ? items.map((item) => (
        <div key={`${title}-${item.id}`} className="constraint-impact-row">
          <StatusBadge value={item.severity || item.family || 'CONSTRAINT'} />
          <span>{item.label || item.id}</span>
          {item.sourceRef && <SourceRefChip refValue={item.sourceRef} />}
          {item.rationale && <small>{item.rationale}</small>}
        </div>
      )) : <p className="muted">{empty}</p>}
    </div>
  );
}

function SourceRefChip({ refValue }: { refValue: string }) {
  const route = sourceRefRoute(refValue);
  const label = sourceRefLabel(refValue);
  const className = `route-source route-source-${label.family.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;
  if (route) {
    return (
      <Link to={route} className={className}>
        <strong>{label.family}</strong>
        {label.id || refValue}
      </Link>
    );
  }
  return (
    <span className={className}>
      <strong>{label.family}</strong>
      {label.id || refValue}
    </span>
  );
}

function TraceRow({ trace }: { trace: RerouteTraceSummary }) {
  return (
    <div className="reroute-trace-row">
      <code>{trace.ruleId || 'RULE'}</code>
      <span>{trace.message || trace.stage || 'Trace step'}</span>
      {trace.sourceRef && <SourceRefChip refValue={trace.sourceRef} />}
    </div>
  );
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="metric mini">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function formatNumber(value?: number) {
  if (value == null || !Number.isFinite(value)) return '0';
  return value >= 100 ? Math.round(value).toLocaleString() : value.toFixed(1);
}

function formatCurrency(value?: number) {
  if (value == null || !Number.isFinite(value)) return '0';
  return Math.round(value).toLocaleString();
}

function formatDecimal(value?: number, digits = 1) {
  if (value == null || !Number.isFinite(value)) return '0';
  return value.toFixed(digits);
}
