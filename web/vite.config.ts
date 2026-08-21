import { defineConfig } from 'vite';
export default defineConfig({base: process.env.FLIGHT_LAB_BASE ?? '/',build:{target:'es2022'},test:{environment:'node',include:['src/**/*.test.ts']}});
