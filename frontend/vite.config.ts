import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';

const apiTarget = process.env.VITE_API_PROXY_TARGET ?? 'http://localhost:8090';

export default defineConfig({
  plugins: [react()],
  test: {
    exclude: ['node_modules/**', 'dist/**', 'e2e/**']
  },
  server: {
    port: 5173,
    proxy: {
      '/api': apiTarget
    }
  }
});
