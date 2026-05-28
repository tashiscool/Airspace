import { Cloud, FileText, Plane, ShieldAlert } from 'lucide-react';
import { Link } from 'react-router-dom';
import { StatusBadge } from './StatusBadge';
import { compactId, fmtZ, type MissionWeatherVerdict, type WeatherGuidanceItem } from '../lib/viewModels';

export function WeatherVerdictPill({ verdict }: { verdict: MissionWeatherVerdict }) {
  return (
    <span className="weather-verdict-pill" title={`${verdict.summary} · confidence ${Math.round(verdict.confidence * 100)}%`}>
      <Cloud size={12} />
      <StatusBadge value={verdict.action} />
      <small>{Math.round(verdict.confidence * 100)}%</small>
    </span>
  );
}

export function WeatherVerdictStrip({ verdict, missionId }: { verdict: MissionWeatherVerdict; missionId?: string }) {
  return (
    <section className={verdict.priority === 'HIGH' ? 'weather-verdict-strip attention-card' : 'weather-verdict-strip'}>
      <div className="verdict-main">
        <span className="verdict-icon"><ShieldAlert size={15} /></span>
        <div>
          <div className="verdict-heading">
            <span>Weather verdict</span>
            <StatusBadge value={verdict.action} />
            {verdict.actionSublabel && <small className="action-sublabel">{verdict.actionSublabel}</small>}
            <small>{Math.round(verdict.confidence * 100)}% confidence</small>
          </div>
          <p>{verdict.summary}</p>
          <small>Recommended: {verdict.recommendedAction}</small>
        </div>
      </div>
      <div className="verdict-sources">
        {verdict.sources.length ? verdict.sources.map((source) => (
          <SourceChip key={source.id} source={source} />
        )) : (
          <span className="muted">No linked weather/PIREP/NOTAM source artifacts.</span>
        )}
      </div>
      <div className="verdict-actions">
        <Link to="/weather">Weather board</Link>
        <Link to={missionId ? `/decisions/latest?mission=${encodeURIComponent(missionId)}` : '/decisions/latest'}>Decision trace</Link>
      </div>
    </section>
  );
}

function SourceChip({ source }: { source: WeatherGuidanceItem }) {
  const Icon = source.sourceFamily.includes('PIREP') ? Plane : source.sourceFamily.includes('NOTAM') ? FileText : Cloud;
  return (
    <span className="source-chip" title={`${source.hazard}: ${source.rationale}`}>
      <Icon size={12} />
      <strong>{source.sourceFamily}</strong>
      <span>{source.sourceLabel || compactId(source.id)}</span>
      <small>{fmtZ(source.createdAt)}</small>
    </span>
  );
}
