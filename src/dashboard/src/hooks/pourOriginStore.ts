/**
 * Cumulative pour origin for the live MES hero.
 *
 * Survives Operations remounts (sessionStorage) and rescoop pour_start re-tares.
 * A rescoop must not raise the origin — that made the hero show ~0 while the
 * vessel still held the first pour. A new weightment key starts a fresh origin
 * so leftover powder on the scale does not count toward the next batch.
 */
const STORAGE_KEY = 'rhapsodi.pourOrigin.v1'

type StoredOrigin = { key: string; originG: number }

let boundKey: string | null = null
let originG: number | null = null

function loadStored(): StoredOrigin | null {
  try {
    const raw = sessionStorage.getItem(STORAGE_KEY)
    if (!raw) return null
    const parsed = JSON.parse(raw) as StoredOrigin
    if (typeof parsed?.originG !== 'number' || !Number.isFinite(parsed.originG)) return null
    if (typeof parsed.key !== 'string' || !parsed.key) return null
    return parsed
  } catch {
    return null
  }
}

function saveStored(rec: StoredOrigin | null): void {
  try {
    if (!rec) sessionStorage.removeItem(STORAGE_KEY)
    else sessionStorage.setItem(STORAGE_KEY, JSON.stringify(rec))
  } catch {
    // sessionStorage unavailable (tests / private mode) — memory still works
  }
}

function remember(key: string, grams: number): void {
  boundKey = key
  originG = grams
  saveStored({ key, originG: grams })
}

export function pourOriginKey(meta: {
  weightment_id?: string | number | null
  run_id?: string | number | null
} | null): string | null {
  if (!meta) return null
  if (meta.weightment_id != null && String(meta.weightment_id).length > 0) {
    return `w:${meta.weightment_id}`
  }
  if (meta.run_id != null && String(meta.run_id).length > 0) {
    return `r:${meta.run_id}`
  }
  return null
}

/**
 * Switch weightment. Same key keeps the origin. Null does not clear it
 * (page remount / mode effect). A different key drops the origin so the next
 * pour latches whatever is on the scale.
 */
export function bindPourOriginKey(key: string | null): boolean {
  if (key == null) return false
  if (key === boundKey) {
    if (originG == null) {
      const saved = loadStored()
      if (saved && saved.key === key) originG = saved.originG
    }
    return false
  }
  // First pour can beat metadata. Attach the anonymous latch to this key.
  if ((boundKey == null || boundKey === 'anon') && originG != null) {
    remember(key, originG)
    return false
  }
  const saved = loadStored()
  if (saved && saved.key === key) {
    boundKey = key
    originG = saved.originG
    return false
  }
  boundKey = key
  originG = null
  saveStored(null)
  return true
}

export function getPourOriginG(): number | null {
  if (originG == null) {
    const saved = loadStored()
    if (saved && (boundKey == null || saved.key === boundKey || saved.key === 'anon')) {
      boundKey = saved.key
      originG = saved.originG
    }
  }
  return originG
}

export function getBoundPourOriginKey(): string | null {
  return boundKey
}

/**
 * Latch vessel weight at the first pour of this weightment.
 * Later pour_start samples (rescoop) cannot raise the origin.
 */
export function latchPourOriginG(weightG: number): number {
  const saved = loadStored()
  if (originG == null && saved && (boundKey == null || saved.key === boundKey || boundKey === 'anon' || saved.key === 'anon')) {
    boundKey = boundKey && boundKey !== 'anon' ? boundKey : saved.key
    originG = saved.originG
  }
  const key = boundKey ?? 'anon'
  if (originG == null) {
    remember(key, weightG)
    return originG
  }
  // Rescoop re-tare arrives at the already-poured weight. Keep the lower origin.
  if (weightG + 1 < originG) {
    remember(key === 'anon' && saved ? saved.key : key, weightG)
  }
  return originG
}

export function clearPourOrigin(): void {
  boundKey = null
  originG = null
  saveStored(null)
}

/** Test helper */
export function _resetPourOriginStoreForTests(): void {
  clearPourOrigin()
}
