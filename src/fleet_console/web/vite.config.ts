import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const pkg = JSON.parse(
  readFileSync(fileURLToPath(new URL('./package.json', import.meta.url)), 'utf-8'),
) as { version: string }

export default defineConfig({
  plugins: [react()],
  define: {
    __APP_VERSION__: JSON.stringify(pkg.version),
  },
  server: {
    port: 5174,
    proxy: {
      '/api': 'http://127.0.0.1:8090',
      '/health': 'http://127.0.0.1:8090',
    },
  },
  build: {
    outDir: '../static',
    emptyOutDir: true,
  },
})
