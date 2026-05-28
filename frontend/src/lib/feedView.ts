import type { FeedTransactionSummary } from '../types';

export function transactionFieldLabel(row: FeedTransactionSummary): string {
  if (row.familySemantic || row.familyLifecycle) {
    return retainedFamilyLabel(row);
  }
  if (row.serviceCommandType || row.serviceCommandService || row.serviceCommandOperation) {
    return serviceCommandLabel(row);
  }
  if (row.domesticNotamKeyword || row.domesticNotamReducerRuleId) {
    return domesticNotamLabel(row);
  }
  if (!row.notamType && !row.notamQCode && !row.notamAffectedLocation && !row.notamAccountability) return '—';
  const parts = [
    row.notamType,
    row.notamQCode,
    row.notamAffectedLocation || row.notamAccountability,
    row.notamHasGeometry ? 'GEOM' : 'NO GEOM',
    row.notamPermanentEnd ? 'PERM' : row.notamEstimatedEnd ? 'EST' : undefined
  ].filter(Boolean);
  return parts.join(' · ');
}

export function transactionMetadataNotice(row: FeedTransactionSummary): string {
  if (row.familySemantic || row.familyLifecycle) {
    return 'Classified legacy traffic retained as typed source-family metadata; no ALTRV reservation, NOTAM geometry, or table mutation is inferred.';
  }
  if (row.serviceCommandType || row.serviceCommandService || row.serviceCommandOperation) {
    if (row.serviceCommandType === 'TABLE') {
      return 'Service table command retained as typed USNS traffic; table mutation is not applied by the local prototype.';
    }
    return 'Service request command retained as typed USNS traffic with parsed location, request flags, and diagnostics.';
  }
  if (row.domesticNotamKeyword || row.domesticNotamReducerRuleId) {
    if (row.domesticNotamReducerRuleId === 'DOM2.UNMATCHED') {
      return 'Domestic NOTAM record retained, but semantic reduction is ambiguous; review raw text and diagnostics.';
    }
    if (isLowVisibilityDomesticNotam(row)) {
      return 'Advisory: Low-visibility/RVR NOTAM retained as an operational constraint; confirm local FAA procedure names, airport protections, and ICAO/operator terminology with authorized sources before using it for departure or taxi planning.';
    }
    if (isApproachMinimaDomesticNotam(row)) {
      return 'Approach/minima NOTAM retained as an operational constraint; confirm landing-aid, lighting, category, and runway visual acquisition impacts with current official sources.';
    }
    if (isSurfaceFrictionDomesticNotam(row)) {
      return 'Runway surface/friction NOTAM retained as an operational constraint; review braking action, friction coefficient, and takeoff/landing performance impact against operator limits.';
    }
    return 'Domestic NOTAM constraint includes DOM1 record metadata and DOM2 semantic reducer output.';
  }
  if (!row.notamType && !row.notamQCode) return 'No NOTAM field metadata retained.';
  if (row.notamHasGeometry) return 'NOTAM constraint includes route-impact geometry metadata.';
  return 'NOTAM constraint retained without map geometry; review raw text and diagnostics.';
}

function retainedFamilyLabel(row: FeedTransactionSummary): string {
  const identifier = row.familyGenotSeries
    || row.familySnowtamId
    || row.familyBirdtamId
    || row.familyAshtamId
    || row.familyFdcId
    || row.familyNotamAction;
  const parts = [
    row.type,
    row.familySemantic,
    row.familyLifecycle,
    identifier
  ].filter(Boolean);
  return parts.length ? parts.join(' · ') : 'RETAINED TRAFFIC';
}

function serviceCommandLabel(row: FeedTransactionSummary): string {
  const flags = [
    row.serviceCommandCount ? 'COUNT' : undefined,
    row.serviceCommandList ? 'LIST' : undefined,
    row.serviceCommandHistory ? 'HISTORY' : undefined,
    row.serviceCommandCurrent ? 'CURRENT' : undefined,
    row.serviceCommandWmscrEcho ? 'WMSCR' : undefined,
    row.serviceCommandPrivilegedHistoryRequest ? 'PRIV-HIST' : undefined
  ].filter(Boolean).join('/');
  const parts = [
    row.serviceCommandType,
    row.serviceCommandDomain,
    row.serviceCommandOperation || row.serviceCommandTableName,
    row.serviceCommandLocation,
    row.serviceCommandNotamId,
    flags || undefined,
    row.serviceCommandAccepted ? 'ACCEPTED' : undefined
  ].filter(Boolean);
  return parts.length ? parts.join(' · ') : 'SERVICE COMMAND';
}

function domesticNotamLabel(row: FeedTransactionSummary): string {
  const qCode = [row.domesticNotamQ23, row.domesticNotamQ45].filter(Boolean).join('/');
  const parts = [
    row.domesticNotamKeyword,
    row.domesticNotamSemanticFacilityFamily,
    row.domesticNotamSemanticCondition || row.domesticNotamSemanticAction,
    row.domesticNotamReducerRuleId,
    qCode || undefined,
    row.domesticNotamUnofficial ? 'UNOFFICIAL' : undefined
  ].filter(Boolean);
  return parts.length ? parts.join(' · ') : 'DOMESTIC NOTAM';
}

function isLowVisibilityDomesticNotam(row: FeedTransactionSummary): boolean {
  return row.domesticNotamReducerRuleId === 'DOM2.SVC.RVR'
    || row.domesticNotamReducerRuleId === 'DOM2.RWY.RVR'
    || row.domesticNotamReducerRuleId === 'DOM2.AD.RVR_ALL'
    || row.domesticNotamReducerRuleId === 'DOM2.SVC.LOW_VISIBILITY_PROCEDURE'
    || row.domesticNotamSemanticCondition === 'RVR_ALL'
    || row.domesticNotamSemanticCondition === 'RVR'
    || row.domesticNotamSemanticCondition === 'RVRM'
    || row.domesticNotamSemanticCondition === 'RVRR'
    || row.domesticNotamSemanticCondition === 'RVRT'
    || row.domesticNotamSemanticCondition === 'LOW_VISIBILITY_PROCEDURE';
}

function isApproachMinimaDomesticNotam(row: FeedTransactionSummary): boolean {
  return row.domesticNotamReducerRuleId === 'DOM2.NAV.APPROACH_MINIMA'
    || row.domesticNotamReducerRuleId === 'DOM2.LIGHTING.APPROACH'
    || row.domesticNotamSemanticCondition === 'APPROACH_MINIMA'
    || row.domesticNotamSemanticCondition === 'APPROACH_LIGHTING';
}

function isSurfaceFrictionDomesticNotam(row: FeedTransactionSummary): boolean {
  return row.domesticNotamReducerRuleId === 'DOM2.SURFACE.FRICTION'
    || row.domesticNotamSemanticCondition === 'FRICTION';
}
