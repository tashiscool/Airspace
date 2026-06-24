import {
  BarChart3,
  Calculator,
  CloudSun,
  Database,
  FlaskConical,
  GitBranch,
  LayoutGrid,
  Mail,
  MonitorPlay,
  Radio,
  ScrollText,
  Search,
  Send,
  Shuffle,
  type LucideIcon
} from 'lucide-react';
import type { WorkbenchSelection } from './workbenchState';

export const WORKBENCH_NAV_LINKS: WorkbenchNavLink[] = [
  { to: '/explorer', icon: LayoutGrid, label: 'Missions', hint: '⌘1' },
  { to: '/deconfliction', icon: Shuffle, label: 'Deconfliction', hint: '⌘2' },
  { to: '/messages', icon: Mail, label: 'Messaging', hint: '⌘3' },
  { to: '/feed', icon: Send, label: 'USNS Feed', hint: '⌘4' },
  { to: '/decisions/latest', icon: GitBranch, label: 'Decisions', hint: '⌘5' },
  { to: '/notams', icon: Radio, label: 'NOTAMs', hint: '⌘6' },
  { to: '/weather', icon: CloudSun, label: 'Weather', hint: '⌘7' },
  { to: '/tfm', icon: BarChart3, label: 'TFM Board', hint: 'flow' },
  { to: '/outcomes', icon: Calculator, label: 'Outcomes', hint: 'metrics' },
  { to: '/simulation', icon: MonitorPlay, label: 'Simulation', hint: '⌘8' },
  { to: '/search', icon: Search, label: 'Search', hint: '⌘9' },
  { to: '/config', icon: Database, label: 'Config', hint: 'settings' },
  { to: '/audit', icon: ScrollText, label: 'Audit', hint: '⌘0' }
];

export type WorkbenchNavLink = {
  to: string;
  icon: LucideIcon;
  label: string;
  hint: string;
};

export type WorkbenchPaletteCommand = {
  id: string;
  label: string;
  route: string;
  hint: string;
  icon: LucideIcon;
};

export type TargetKind = 'mission' | 'reservation' | 'message' | 'feed' | 'decision';

export function buildWorkbenchCommands(selection: WorkbenchSelection = {}): WorkbenchPaletteCommand[] {
  return [
    ...WORKBENCH_NAV_LINKS.map((link) => ({ id: link.to, label: `Open ${link.label}`, route: link.to, hint: link.to, icon: link.icon })),
    { id: 'compose-message', label: 'Compose message', route: '/messages', hint: 'USNS / internal message', icon: Mail },
    { id: 'ingest-feed', label: 'Ingest feed payload', route: '/feed', hint: 'USNS/weather/PIREP/NOTAM traffic', icon: Send },
    { id: 'run-simulation', label: 'Run simulation scenario', route: '/simulation', hint: 'timeline, route impact, pilot brief', icon: MonitorPlay },
    { id: 'open-tfm-board', label: 'Open TFM command-center board', route: '/tfm', hint: 'airport demand, sector load, TMIs, route alternatives', icon: BarChart3 },
    { id: 'open-outcomes', label: 'Open outcome metrics', route: '/outcomes', hint: 'delay, fuel, overload, false-clear, source refs', icon: Calculator },
    { id: 'author-simulation-scenario', label: 'Author simulation scenario', route: '/simulation/author', hint: 'validate, import, red-team drafts', icon: FlaskConical },
    { id: 'review-notams', label: 'Review NOTAM constraints', route: '/notams', hint: 'constraints, not reservations', icon: Radio },
    {
      id: 'review-conflicts',
      label: 'Review selected reservation conflicts',
      route: selection.reservationId ? `/deconfliction/${selection.reservationId}` : '/deconfliction',
      hint: 'deconfliction workbench',
      icon: Shuffle
    },
    {
      id: 'verify-replay',
      label: 'Open decision replay',
      route: selection.decisionId ? `/decisions/${selection.decisionId}` : '/decisions/latest',
      hint: 'audit/replay',
      icon: GitBranch
    }
  ];
}

export function filterWorkbenchCommands(commands: WorkbenchPaletteCommand[], query: string) {
  const q = query.trim().toLowerCase();
  return commands.filter((command) => !q || `${command.label} ${command.hint} ${command.route}`.toLowerCase().includes(q));
}

export function nextPaletteIndex(current: number, direction: 1 | -1, total: number) {
  if (total <= 0) return -1;
  if (current < 0) return direction === 1 ? 0 : total - 1;
  return (current + direction + total) % total;
}

export function routeForTarget(kind: TargetKind, id: string) {
  const targetId = id.trim();
  if (!targetId) return undefined;
  if (kind === 'mission') return `/missions/${targetId}`;
  if (kind === 'reservation') return `/deconfliction/${targetId}`;
  if (kind === 'message') return `/messages/${targetId}`;
  if (kind === 'feed') return `/feed/${targetId}`;
  return `/decisions/${targetId}`;
}

export function routesForSelection(selection: WorkbenchSelection = {}): Array<{ label: string; route: string }> {
  const routes: Array<{ label: string; route: string }> = [];
  if (selection.missionId) routes.push({ label: 'Mission', route: `/missions/${selection.missionId}` });
  if (selection.reservationId) routes.push({ label: 'Reservation', route: `/deconfliction/${selection.reservationId}` });
  if (selection.messageId) routes.push({ label: 'Message', route: `/messages/${selection.messageId}` });
  if (selection.feedArtifactId) routes.push({ label: 'Feed', route: `/feed/${selection.feedArtifactId}` });
  if (selection.decisionId) routes.push({ label: 'Decision', route: `/decisions/${selection.decisionId}` });
  if (selection.sourceFamily === 'NOTAM') routes.push({ label: 'NOTAMs', route: '/notams' });
  if (selection.sourceFamily === 'WEATHER' || selection.sourceFamily === 'PIREP') routes.push({ label: 'Weather', route: '/weather' });
  if (selection.sourceFamily === 'TFM') routes.push({ label: 'TFM Board', route: '/tfm' });
  if (selection.sourceFamily === 'OUTCOME') routes.push({ label: 'Outcome Metrics', route: '/outcomes' });
  return dedupeRoutes(routes);
}

function dedupeRoutes(routes: Array<{ label: string; route: string }>) {
  const seen = new Set<string>();
  return routes.filter((item) => {
    if (seen.has(item.route)) return false;
    seen.add(item.route);
    return true;
  });
}
