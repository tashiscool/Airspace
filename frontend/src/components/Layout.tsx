import { useEffect, useMemo, useState } from 'react';
import { NavLink, Outlet, useLocation, useNavigate } from 'react-router-dom';
import { useQuery } from '@tanstack/react-query';
import { ChevronRight, Clock, LogOut, PanelLeftClose, RadioTower, Search } from 'lucide-react';
import { api } from '../api/client';
import { useSession } from '../state/session';
import {
  buildWorkbenchCommands,
  filterWorkbenchCommands,
  nextPaletteIndex,
  routeForTarget,
  routesForSelection,
  WORKBENCH_NAV_LINKS,
  type TargetKind
} from '../lib/workbenchCommands';
import {
  DEFAULT_LAYOUT_PREFS,
  readWorkbenchJson,
  sourceFamilyLabel,
  type WorkbenchLayoutPrefs,
  type WorkbenchSelection,
  writeWorkbenchJson
} from '../lib/workbenchState';

export function Layout() {
  const user = useSession((state) => state.user);
  const clear = useSession((state) => state.clear);
  const navigate = useNavigate();
  const location = useLocation();
  const [prefs, setPrefs] = useState<WorkbenchLayoutPrefs>(() => readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS));
  const [paletteOpen, setPaletteOpen] = useState(false);
  const [selection, setSelection] = useState<WorkbenchSelection>(() => readWorkbenchJson('airspace.workbench.selection', {}));

  useEffect(() => {
    writeWorkbenchJson('airspace.workbench.layout', prefs);
  }, [prefs]);

  useEffect(() => {
    const refreshSelection = () => setSelection(readWorkbenchJson('airspace.workbench.selection', {}));
    window.addEventListener('storage', refreshSelection);
    window.addEventListener('airspace-workbench-selection', refreshSelection);
    refreshSelection();
    return () => {
      window.removeEventListener('storage', refreshSelection);
      window.removeEventListener('airspace-workbench-selection', refreshSelection);
    };
  }, [location.pathname]);

  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === 'k') {
        event.preventDefault();
        setPaletteOpen((open) => !open);
      }
      if ((event.metaKey || event.ctrlKey) && /^[0-9]$/.test(event.key)) {
        const index = event.key === '0' ? 9 : Number(event.key) - 1;
        const target = WORKBENCH_NAV_LINKS[index];
        if (target) {
          event.preventDefault();
          navigate(target.to);
        }
      }
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [navigate]);

  return (
    <div className="ops-shell">
      <header className="ops-topbar">
        <div className="ops-brand">
          <span className="brand-mark"><RadioTower size={15} /></span>
          <strong>Airspace Workbench</strong>
          <span className="env-pill">FAA Ops</span>
          <Breadcrumb path={location.pathname} />
        </div>
        <button
          className="command-button"
          onPointerDown={(event) => {
            event.preventDefault();
            setPaletteOpen(true);
          }}
          onClick={() => window.setTimeout(() => setPaletteOpen(true), 0)}
          aria-label="Open command palette"
        >
          <Search size={15} />
          <span>Search missions, reservations, NOTAMs...</span>
          <kbd>⌘</kbd><kbd>K</kbd>
        </button>
        <div className="operator-block">
          <UtcClock />
          <span className="operator-avatar">{(user?.displayName ?? user?.username ?? '?').slice(0, 1)}</span>
          <span>
            <strong>{user?.displayName ?? 'Local operator'}</strong>
            <small>{user?.roles?.join(', ') || 'planner'}</small>
          </span>
          <button className="icon-button" onClick={clear} title="Sign out" aria-label="Sign out"><LogOut size={15} /></button>
        </div>
      </header>
      <ContextStrip selection={selection} />
      <div className="ops-body">
        <nav className={prefs.railCollapsed ? 'ops-rail collapsed' : 'ops-rail'} aria-label="Primary">
          {WORKBENCH_NAV_LINKS.map((link) => (
            <NavLink
              key={link.to}
              to={link.to}
              className={({ isActive }) => (isActive || location.pathname.startsWith(link.to.replace('/latest', '')) ? 'active' : '')}
              title={link.label}
            >
              <link.icon size={17} />
              {!prefs.railCollapsed && <span>{link.label}</span>}
              {!prefs.railCollapsed && <small>{link.hint}</small>}
            </NavLink>
          ))}
          <button className="rail-collapse" onClick={() => setPrefs((value) => ({ ...value, railCollapsed: !value.railCollapsed }))}>
            <PanelLeftClose size={15} />
            {!prefs.railCollapsed && 'Collapse'}
          </button>
        </nav>
        <main className="ops-main">
          <Outlet />
          <footer className="ops-footer">
            <OperationalSummary />
            <span>CARF/ALTRV, NOTAM, weather, and PIREP sources stay distinct until fused by the engine.</span>
          </footer>
        </main>
      </div>
      {paletteOpen && <CommandPalette onClose={() => setPaletteOpen(false)} />}
    </div>
  );
}

function Breadcrumb({ path }: { path: string }) {
  const parts = path.split('/').filter(Boolean);
  const label = parts[0] === 'explorer' ? 'Mission Explorer' : parts[0] ? titleCase(parts[0]) : 'Mission Explorer';
  return (
    <nav className="breadcrumb" aria-label="breadcrumb">
      <span>Airspace</span>
      <ChevronRight size={13} />
      <strong>{label}</strong>
      {parts.slice(1).map((part) => (
        <span key={part} className="breadcrumb-part"><ChevronRight size={13} />{part}</span>
      ))}
    </nav>
  );
}

function ContextStrip({ selection }: { selection: WorkbenchSelection }) {
  const navigate = useNavigate();
  const feed = useQuery({ queryKey: ['feed-artifacts', 'context'], queryFn: api.feedArtifacts, staleTime: 30_000 });
  const messages = useQuery({ queryKey: ['messages', 'context'], queryFn: api.messages, staleTime: 30_000 });
  const rejectedFeed = (feed.data ?? []).filter((item) => !item.accepted).length;
  const weatherAlerts = (messages.data ?? []).filter((message) => /PIREP|SIGMET|AIRMET|METAR|TAF|WEATHER/i.test(message.family)).length;
  const notamAlerts = (messages.data ?? []).filter((message) => /NOTAM|DOM|FDC|SNOWTAM|BIRDTAM|ASHTAM|GENOT/i.test(message.family)).length;
  return (
    <div className="context-strip" aria-label="Current operational context">
      <span><strong>Context</strong> {selection.label ?? 'No active object'}</span>
      <span>{sourceFamilyLabel(selection.sourceFamily)}</span>
      <span>{selection.missionId ? `Mission ${selection.missionId}` : 'Mission -'}</span>
      <span>{selection.reservationId ? `Reservation ${selection.reservationId}` : 'Reservation -'}</span>
      <span className={selection.conflictCount ? 'attention' : ''}>{selection.conflictCount ?? 0} conflicts</span>
      <span>{selection.lockState ?? 'Unlocked/unknown'}</span>
      <span className={rejectedFeed ? 'attention' : ''}>{rejectedFeed} rejected feed</span>
      <span>{weatherAlerts} weather/PIREP</span>
      <span>{notamAlerts} NOTAM</span>
      {routesForSelection(selection).map((item) => (
        <button key={item.route} className="context-action" onClick={() => navigate(item.route)}>{item.label}</button>
      ))}
      {(selection.label || selection.missionId || selection.reservationId || selection.messageId || selection.feedArtifactId || selection.decisionId) && (
        <button
          className="context-action muted-action"
          onClick={() => {
            writeWorkbenchJson('airspace.workbench.selection', {});
            window.dispatchEvent(new Event('airspace-workbench-selection'));
          }}
        >
          Clear
        </button>
      )}
    </div>
  );
}

function UtcClock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const id = window.setInterval(() => setNow(new Date()), 1000);
    return () => window.clearInterval(id);
  }, []);
  return <span className="utc-clock"><Clock size={14} />{now.toISOString().slice(11, 19)}Z</span>;
}

function OperationalSummary() {
  const missions = useQuery({ queryKey: ['missions', 'shell'], queryFn: api.missions, staleTime: 30_000 });
  const messages = useQuery({ queryKey: ['messages', 'shell'], queryFn: api.messages, staleTime: 30_000 });
  const feed = useQuery({ queryKey: ['feed-artifacts', 'shell'], queryFn: api.feedArtifacts, staleTime: 30_000 });
  const history = useQuery({ queryKey: ['history', 'shell'], queryFn: api.history, staleTime: 30_000 });
  const rejectedFeed = (feed.data ?? []).filter((item) => !item.accepted).length;
  const missionCount = missions.data?.length ?? 0;
  const messageCount = messages.data?.length ?? 0;
  return (
    <span className="footer-summary">
      <span className="live-dot" /> Link OK
      <span>{missionCount} missions</span>
      <span>{messageCount} messages</span>
      <span className={rejectedFeed ? 'attention' : ''}>{rejectedFeed} rejected feed</span>
      <span>{history.data?.length ?? 0} audit events</span>
    </span>
  );
}

function CommandPalette({ onClose }: { onClose: () => void }) {
  const navigate = useNavigate();
  const [query, setQuery] = useState('');
  const [targetId, setTargetId] = useState('');
  const [selectedIndex, setSelectedIndex] = useState(0);
  const selection = readWorkbenchJson<WorkbenchSelection>('airspace.workbench.selection', {});
  const commands = useMemo(() => buildWorkbenchCommands(selection), [selection.decisionId, selection.reservationId]);
  const filtered = useMemo(() => filterWorkbenchCommands(commands, query), [commands, query]);
  useEffect(() => {
    setSelectedIndex(filtered.length ? 0 : -1);
  }, [filtered.length, query]);

  function openCommand(route?: string) {
    if (!route) return;
    navigate(route);
    onClose();
  }

  function openTypedTarget(kind: TargetKind) {
    openCommand(routeForTarget(kind, targetId));
  }
  return (
    <div className="palette-backdrop" role="presentation">
      <div className="command-palette" role="dialog" aria-label="Command palette">
        <div className="palette-header">
          <input
            autoFocus
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === 'Escape') onClose();
              if (event.key === 'ArrowDown' || event.key === 'ArrowUp') {
                event.preventDefault();
                setSelectedIndex((current) => nextPaletteIndex(current, event.key === 'ArrowDown' ? 1 : -1, filtered.length));
              }
              if (event.key === 'Enter' && filtered[0]) {
                navigate(filtered[Math.max(0, selectedIndex)]?.route ?? filtered[0].route);
                onClose();
              }
            }}
            placeholder="Navigate, launch workflow actions, or search commands..."
          />
          <button className="palette-close" onClick={onClose} aria-label="Close command palette">Close</button>
        </div>
        <div className="target-jump">
          <input
            value={targetId}
            onChange={(event) => setTargetId(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === 'Escape') onClose();
              if (event.key === 'Enter') openTypedTarget('mission');
            }}
            placeholder="Open by ID..."
          />
          <button onClick={() => openTypedTarget('mission')}>Mission</button>
          <button onClick={() => openTypedTarget('reservation')}>Reservation</button>
          <button onClick={() => openTypedTarget('message')}>Message</button>
          <button onClick={() => openTypedTarget('feed')}>Feed</button>
          <button onClick={() => openTypedTarget('decision')}>Decision</button>
        </div>
        <div>
          {filtered.map((item, index) => (
            <button
              key={item.id}
              className={index === selectedIndex ? 'palette-command active' : 'palette-command'}
              onClick={() => {
                openCommand(item.route);
              }}
            >
              <item.icon size={15} />
              <span>{item.label}</span>
              <small>{item.hint}</small>
            </button>
          ))}
        </div>
        <div className="palette-footer">
          <span>Enter opens highlighted command</span>
          <span>Target ID + Enter opens Mission</span>
          <span>Esc closes</span>
        </div>
      </div>
    </div>
  );
}

function titleCase(value: string) {
  return value.replace(/-/g, ' ').replace(/\b\w/g, (char) => char.toUpperCase());
}
