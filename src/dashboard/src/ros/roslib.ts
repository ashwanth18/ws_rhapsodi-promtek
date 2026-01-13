// Centralized access to the global ROSLIB object.
//
// We intentionally load roslibjs via a <script> tag in index.html (classic script),
// because bundling roslib (CommonJS) through Vite/Rollup can break due to its
// use of top-level `this`.

export const ROSLIB: any = (globalThis as any).ROSLIB


