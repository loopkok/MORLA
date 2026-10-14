import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Dev: Vite on :5173 proxies /api and /ws to the FastAPI backend on :8080.
// Prod: `npm run build` emits dist/, served statically by FastAPI.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      '/api': { target: 'http://127.0.0.1:8080', changeOrigin: true },
      '/ws': { target: 'ws://127.0.0.1:8080', ws: true },
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
})
