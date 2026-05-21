import { Component, lazy, Suspense, type ErrorInfo, type ReactNode } from 'react';
import { Navigate, Route, Routes } from 'react-router-dom';
import { Layout } from './components/Layout';
import { useSession } from './state/session';

const LoginPage = lazy(() => import('./pages/LoginPage').then((module) => ({ default: module.LoginPage })));
const ExplorerPage = lazy(() => import('./pages/ExplorerPage').then((module) => ({ default: module.ExplorerPage })));
const MissionPage = lazy(() => import('./pages/MissionPage').then((module) => ({ default: module.MissionPage })));
const ReservationPage = lazy(() => import('./pages/ReservationPage').then((module) => ({ default: module.ReservationPage })));
const DecisionPage = lazy(() => import('./pages/DecisionPage').then((module) => ({ default: module.DecisionPage })));
const MessagesPage = lazy(() => import('./pages/MessagesPage').then((module) => ({ default: module.MessagesPage })));
const DeconflictionPage = lazy(() => import('./pages/DeconflictionPage').then((module) => ({ default: module.DeconflictionPage })));
const FeedPage = lazy(() => import('./pages/FeedPage').then((module) => ({ default: module.FeedPage })));
const SearchPage = lazy(() => import('./pages/SearchPage').then((module) => ({ default: module.SearchPage })));
const HistoryPage = lazy(() => import('./pages/HistoryPage').then((module) => ({ default: module.HistoryPage })));
const ConfigPage = lazy(() => import('./pages/ConfigPage').then((module) => ({ default: module.ConfigPage })));
const NotamsPage = lazy(() => import('./pages/NotamsPage').then((module) => ({ default: module.NotamsPage })));
const WeatherPage = lazy(() => import('./pages/WeatherPage').then((module) => ({ default: module.WeatherPage })));

function Guarded() {
  const token = useSession((state) => state.token);
  if (!token) return <Navigate to="/login" replace />;
  return <Layout />;
}

export function App() {
  return (
    <WorkspaceErrorBoundary>
      <Suspense
        fallback={
          <div className="workspace">
            <p className="muted">Loading workspace...</p>
          </div>
        }
      >
        <Routes>
          <Route path="/login" element={<LoginPage />} />
          <Route element={<Guarded />}>
            <Route path="/explorer" element={<ExplorerPage />} />
            <Route path="/missions/:missionId" element={<MissionPage />} />
            <Route path="/missions/:missionId/reservations/:reservationId" element={<ReservationPage />} />
            <Route path="/deconfliction" element={<DeconflictionPage />} />
            <Route path="/deconfliction/:reservationId" element={<DeconflictionPage />} />
            <Route path="/messages" element={<MessagesPage />} />
            <Route path="/messages/:messageId" element={<MessagesPage />} />
            <Route path="/feed" element={<FeedPage />} />
            <Route path="/feed/:artifactId" element={<FeedPage />} />
            <Route path="/decisions/latest" element={<DecisionPage />} />
            <Route path="/decisions/:decisionId" element={<DecisionPage />} />
            <Route path="/notams" element={<NotamsPage />} />
            <Route path="/weather" element={<WeatherPage />} />
            <Route path="/search" element={<SearchPage />} />
            <Route path="/history" element={<HistoryPage />} />
            <Route path="/audit" element={<HistoryPage />} />
            <Route path="/config" element={<ConfigPage />} />
            <Route path="*" element={<Navigate to="/explorer" replace />} />
          </Route>
          <Route path="*" element={<Navigate to="/login" replace />} />
        </Routes>
      </Suspense>
    </WorkspaceErrorBoundary>
  );
}

class WorkspaceErrorBoundary extends Component<{ children: ReactNode }, { error?: Error }> {
  state: { error?: Error } = {};

  static getDerivedStateFromError(error: Error) {
    return { error };
  }

  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error('Workspace failed to render', error, info.componentStack);
  }

  render() {
    if (this.state.error) {
      return (
        <main className="login-screen">
          <section className="login-panel">
            <h1>Workspace Error</h1>
            <p className="error">{this.state.error.message}</p>
            <button onClick={() => window.location.reload()}>Reload</button>
          </section>
        </main>
      );
    }
    return this.props.children;
  }
}
