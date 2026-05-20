import { create } from 'zustand';
import type { UserSummary } from '../types';

type SessionState = {
  user: UserSummary | null;
  token: string | null;
  setSession: (token: string, user: UserSummary) => void;
  clear: () => void;
};

export const useSession = create<SessionState>((set) => ({
  user: null,
  token: localStorage.getItem('airspace.token'),
  setSession: (token, user) => {
    localStorage.setItem('airspace.token', token);
    set({ token, user });
  },
  clear: () => {
    localStorage.removeItem('airspace.token');
    set({ token: null, user: null });
  }
}));
