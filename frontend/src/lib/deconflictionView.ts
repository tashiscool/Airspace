import type { AirspaceFeature } from '../types';

export type ConflictMetricStatus = 'CLEAR' | 'LIMITED' | 'VIOLATION' | 'UNKNOWN';

export type ConflictMetric = {
  label: string;
  value: number | undefined;
  unit: string;
  threshold: number | undefined;
  status: ConflictMetricStatus;
  percent: number;
};

export type ConflictReviewSummary = {
  id: string;
  severity: string;
  explanation: string;
  metrics: ConflictMetric[];
  sourceRatios: string[];
  sourcePoints: string[];
};

export function conflictReviewSummary(feature?: AirspaceFeature): ConflictReviewSummary | undefined {
  if (!feature) return undefined;
  const props = feature.properties ?? {};
  const lateral = numeric(props.minimumLateralDistanceNauticalMiles ?? props.lateral ?? props.lateralNm);
  const requiredLateral = numeric(props.minimumRequiredLateralNauticalMiles ?? props.requiredLateralNm ?? props.lateralThresholdNm) ?? 60;
  const vertical = numeric(props.verticalSeparationFeet ?? props.vertical ?? props.verticalFeet);
  const requiredVertical = numeric(props.minimumRequiredVerticalFeet ?? props.requiredVerticalFeet ?? props.verticalThresholdFeet) ?? 1000;
  const duration = numeric(props.durationMinutes ?? props.overlapDurationMinutes ?? props.belowMinimumDurationMinutes);
  const requiredDuration = numeric(props.minimumRequiredDurationMinutes ?? props.durationThresholdMinutes) ?? 0;
  const longitudinal = numeric(props.longitudinalSeparationMinutes ?? props.longitudinalMinutes);
  const requiredLongitudinal = numeric(props.minimumRequiredLongitudinalMinutes ?? props.longitudinalThresholdMinutes);

  return {
    id: String(feature.id ?? props.id ?? props.label ?? 'conflict'),
    severity: String(props.severity ?? '').toUpperCase() || severityFromMetrics(lateral, requiredLateral, vertical, requiredVertical),
    explanation: String(props.explanation ?? props.rationale ?? 'No retained conflict explanation.'),
    metrics: [
      separationMetric('Lateral', lateral, 'NM', requiredLateral),
      separationMetric('Vertical', vertical, 'FT', requiredVertical),
      overlapMetric('Time', duration, 'MIN', requiredDuration),
      separationMetric('Longitudinal', longitudinal, 'MIN', requiredLongitudinal)
    ],
    sourceRatios: stringList(props.sourceRatios ?? props.ratios ?? props.routeRatios),
    sourcePoints: stringList(props.sourcePoints ?? props.points ?? props.sourceFixes)
  };
}

function separationMetric(label: string, value: number | undefined, unit: string, threshold: number | undefined): ConflictMetric {
  if (value == null || threshold == null) return { label, value, unit, threshold, status: 'UNKNOWN', percent: 0 };
  const ratio = threshold <= 0 ? 1 : value / threshold;
  return {
    label,
    value,
    unit,
    threshold,
    status: ratio >= 1 ? 'CLEAR' : ratio >= 0.6 ? 'LIMITED' : 'VIOLATION',
    percent: clamp(ratio * 100)
  };
}

function overlapMetric(label: string, value: number | undefined, unit: string, threshold: number | undefined): ConflictMetric {
  if (value == null) return { label, value, unit, threshold, status: 'UNKNOWN', percent: 0 };
  const active = value > (threshold ?? 0);
  return {
    label,
    value,
    unit,
    threshold,
    status: active ? 'VIOLATION' : 'CLEAR',
    percent: clamp(active ? 100 : 0)
  };
}

function severityFromMetrics(
  lateral: number | undefined,
  requiredLateral: number,
  vertical: number | undefined,
  requiredVertical: number
) {
  if ((lateral != null && lateral < requiredLateral * 0.6) || (vertical != null && vertical < requiredVertical * 0.6)) return 'HIGH';
  if ((lateral != null && lateral < requiredLateral) || (vertical != null && vertical < requiredVertical)) return 'MEDIUM';
  return 'LOW';
}

function numeric(value: unknown) {
  if (typeof value === 'number' && Number.isFinite(value)) return value;
  if (typeof value === 'string') {
    const parsed = Number(value.replace(/[^\d.-]/g, ''));
    return Number.isFinite(parsed) ? parsed : undefined;
  }
  return undefined;
}

function stringList(value: unknown) {
  if (Array.isArray(value)) return value.map(String).filter(Boolean).slice(0, 8);
  if (typeof value === 'string' && value.trim()) return value.split(/[,;|]/).map((item) => item.trim()).filter(Boolean).slice(0, 8);
  return [];
}

function clamp(value: number) {
  return Math.max(0, Math.min(100, value));
}
