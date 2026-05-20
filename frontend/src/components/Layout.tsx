import { NavLink, Outlet } from 'react-router-dom';
import { useSession } from '../state/session';

const links = [
  ['/explorer', 'Explorer'],
  ['/messages/inbox', 'Messages'],
  ['/feed', 'Feed'],
  ['/decisions/latest', 'Decisions'],
  ['/search', 'Search'],
  ['/history', 'History'],
  ['/config', 'Config']
];

export function Layout() {
  const user = useSession((state) => state.user);
  const clear = useSession((state) => state.clear);
  return (
    <div className="app-shell">
      <aside className="nav">
        <div className="brand">Airspace Ops</div>
        {links.map(([to, label]) => (
          <NavLink key={to} to={to} className={({ isActive }) => (isActive ? 'active' : '')}>
            {label}
          </NavLink>
        ))}
        <button className="secondary" onClick={clear}>Sign out</button>
      </aside>
      <main>
        <header className="topbar">
          <div>Operations Control</div>
          <div>{user?.displayName ?? 'Local operator'}</div>
        </header>
        <Outlet />
      </main>
    </div>
  );
}
