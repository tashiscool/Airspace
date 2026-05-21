export type WorkbenchCapabilityStatus = 'complete' | 'partial' | 'missing';

export type WorkbenchCapability = {
  id: string;
  label: string;
  status: WorkbenchCapabilityStatus;
  note: string;
};

export const WORKBENCH_READINESS: WorkbenchCapability[] = [
  {
    id: 'shell-context',
    label: 'Persistent shell and operational context',
    status: 'complete',
    note: 'Shared rail, top bar, footer counts, context strip, quick actions, keyboard command catalog, and layout preferences are implemented.'
  },
  {
    id: 'mission-explorer',
    label: 'Mission Explorer operations desk',
    status: 'complete',
    note: 'Four-pane mission/message/table/preview workflow, attention filters, keyboard navigation, row selection, and preview actions are implemented.'
  },
  {
    id: 'reservation-editor',
    label: 'Reservation Sections A-G editor',
    status: 'partial',
    note: 'Sections/raw modes, sticky workflow actions, frontend section checks, supplements, diagnostics, force reasons, map preview, browser unload warnings, and in-app navigation guarding exist; backend-persisted draft conflict/review state remains.'
  },
  {
    id: 'deconfliction-review',
    label: 'Deconfliction review bench',
    status: 'partial',
    note: 'Conflict table, selected detail, filters, console, local reload-safe acceptance state, metric summaries, force reason, and selected map overlay exist; backend-persisted conflict review workflow remains.'
  },
  {
    id: 'map-instrument',
    label: 'Map and visualization instrument',
    status: 'partial',
    note: 'GeoJSON layer groups, fixed group toggles, counts, operational feature summaries, controlled selected feature, OpenLayers click selection, fit all/selected, forecast slider, and layer prefs exist; broader page-level map/table coupling remains.'
  },
  {
    id: 'messaging-feed',
    label: 'Messaging and USNS/feed queues',
    status: 'partial',
    note: 'Inbox/outbox/archive, family chips, reply/forward/compose, recipients text, attachment metadata, related traffic, feed artifact details, and transaction tables exist; recipient directory integration and persisted threading remain shallow.'
  },
  {
    id: 'decision-audit',
    label: 'Decision audit and replay workbench',
    status: 'partial',
    note: 'Decision evaluate/open, search-backed queue, grouped/text/stage filtered trace, constraints, replay, audit JSON, and map tabs exist; richer persisted decision list APIs remain.'
  },
  {
    id: 'notam-weather',
    label: 'NOTAM, weather, and PIREP constraint surfaces',
    status: 'partial',
    note: 'Separate NOTAM and weather/PIREP pages preserve source-family semantics; richer operational review queues and map coupling remain.'
  },
  {
    id: 'config-admin',
    label: 'Config and reference administration',
    status: 'partial',
    note: 'Reference points, imports, local RBAC matrix, recipients, airspace groups, and separation previews exist; live user/recipient CRUD is not fully backed.'
  },
  {
    id: 'frontend-tests',
    label: 'Frontend automated coverage',
    status: 'partial',
    note: 'Build, API, view-model, map-layer, navigation guard, decision/deconfliction view, workbench state, and command tests exist; component tests and browser E2E tests remain.'
  }
];

export function workbenchReadinessSummary(capabilities: WorkbenchCapability[] = WORKBENCH_READINESS) {
  const total = capabilities.length;
  const complete = capabilities.filter((item) => item.status === 'complete').length;
  const partial = capabilities.filter((item) => item.status === 'partial').length;
  const missing = capabilities.filter((item) => item.status === 'missing').length;
  const weighted = capabilities.reduce((sum, item) => sum + (item.status === 'complete' ? 1 : item.status === 'partial' ? 0.55 : 0), 0);
  return {
    total,
    complete,
    partial,
    missing,
    percent: Math.round((weighted / total) * 100)
  };
}

export function remainingWorkbenchGaps(capabilities: WorkbenchCapability[] = WORKBENCH_READINESS) {
  return capabilities.filter((item) => item.status !== 'complete');
}
