import { lazy, Suspense } from 'react';
import { Navigate, Route, Routes } from 'react-router-dom';
import { Layout } from './components/Layout';
import { useSession } from './state/session';

const LoginPage = lazy(() => import('./pages/LoginPage').then((module) => ({ default: module.LoginPage })));
const ExplorerPage = lazy(() => import('./pages/ExplorerPage').then((module) => ({ default: module.ExplorerPage })));
const MissionPage = lazy(() => import('./pages/MissionPage').then((module) => ({ default: module.MissionPage })));
const ReservationPage = lazy(() => import('./pages/ReservationPage').then((module) => ({ default: module.ReservationPage })));
const DecisionPage = lazy(() => import('./pages/DecisionPage').then((module) => ({ default: module.DecisionPage })));
const MessagesPage = lazy(() => import('./pages/MessagesPage').then((module) => ({ default: module.MessagesPage })));
const DeconflictionPage = lazy(() =>
  import('./pages/SimplePages').then((module) => ({ default: module.DeconflictionPage }))
);
const FeedPage = lazy(() => import('./pages/SimplePages').then((module) => ({ default: module.FeedPage })));
const SearchPage = lazy(() => import('./pages/SimplePages').then((module) => ({ default: module.SearchPage })));
const HistoryPage = lazy(() => import('./pages/SimplePages').then((module) => ({ default: module.HistoryPage })));
const ConfigPage = lazy(() => import('./pages/SimplePages').then((module) => ({ default: module.ConfigPage })));

function Guarded() {
  const token = useSession((state) => state.token);
  if (!token) return <Navigate to="/login" replace />;
  return <Layout />;
}

export function App() {
  return (
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
          <Route path="/deconfliction/:reservationId" element={<DeconflictionPage />} />
          <Route path="/messages/:messageId" element={<MessagesPage />} />
          <Route path="/feed" element={<FeedPage />} />
          <Route path="/feed/:artifactId" element={<FeedPage />} />
          <Route path="/decisions/:decisionId" element={<DecisionPage />} />
          <Route path="/search" element={<SearchPage />} />
          <Route path="/history" element={<HistoryPage />} />
          <Route path="/config" element={<ConfigPage />} />
          <Route path="*" element={<Navigate to="/explorer" replace />} />
        </Route>
        <Route path="*" element={<Navigate to="/login" replace />} />
      </Routes>
    </Suspense>
  );
}
