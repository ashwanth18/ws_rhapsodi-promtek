/** Fleet Console SemVer from `web/package.json` (baked at build time). */
export const APP_VERSION =
  typeof __APP_VERSION__ !== 'undefined' ? __APP_VERSION__ : '0.1.0'
