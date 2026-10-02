import { beforeEach, describe, expect, it } from 'vitest'
import {
  _resetPourOriginStoreForTests,
  bindPourOriginKey,
  getPourOriginG,
  latchPourOriginG,
  pourOriginKey,
} from './pourOriginStore'

function installMemoryStorage() {
  const mem = new Map<string, string>()
  const storage = {
    getItem: (k: string) => (mem.has(k) ? mem.get(k)! : null),
    setItem: (k: string, v: string) => {
      mem.set(k, String(v))
    },
    removeItem: (k: string) => {
      mem.delete(k)
    },
  }
  Object.defineProperty(globalThis, 'sessionStorage', {
    configurable: true,
    value: storage,
  })
}

describe('pourOriginStore', () => {
  beforeEach(() => {
    installMemoryStorage()
    _resetPourOriginStoreForTests()
  })

  it('builds a stable key from weightment_id', () => {
    expect(pourOriginKey({ weightment_id: '42', run_id: 'r1' })).toBe('w:42')
    expect(pourOriginKey({ run_id: 'r1' })).toBe('r:r1')
  })

  it('latches origin once and ignores a higher rescoop pour_start', () => {
    bindPourOriginKey('w:1')
    expect(latchPourOriginG(0.5)).toBe(0.5)
    expect(latchPourOriginG(427)).toBe(0.5)
    expect(getPourOriginG()).toBe(0.5)
  })

  it('clears origin only when weightment key changes', () => {
    bindPourOriginKey('w:1')
    latchPourOriginG(10)
    expect(bindPourOriginKey('w:1')).toBe(false)
    expect(getPourOriginG()).toBe(10)
    expect(bindPourOriginKey('w:2')).toBe(true)
    expect(getPourOriginG()).toBe(null)
  })

  it('does not clear origin when the key is unbound', () => {
    bindPourOriginKey('w:1')
    latchPourOriginG(0.5)
    expect(bindPourOriginKey(null)).toBe(false)
    expect(getPourOriginG()).toBe(0.5)
    expect(latchPourOriginG(427)).toBe(0.5)
  })

  it('promotes anonymous latch when metadata arrives late', () => {
    latchPourOriginG(12)
    expect(bindPourOriginKey('w:9')).toBe(false)
    expect(getPourOriginG()).toBe(12)
  })

  it('restores the low origin from sessionStorage after a memory wipe', () => {
    bindPourOriginKey('w:19')
    latchPourOriginG(0.5)
    // Simulate a remount that dropped module state but kept sessionStorage.
    boundKeyWipe()
    expect(latchPourOriginG(427)).toBe(0.5)
  })
})

function boundKeyWipe() {
  // Re-import path: clear only the module fields via a new-key bind that we
  // immediately undo by writing storage back... simpler: call internal by
  // resetting module then relying on storage.
  const raw = sessionStorage.getItem('rhapsodi.pourOrigin.v1')
  _resetPourOriginStoreForTests()
  if (raw) sessionStorage.setItem('rhapsodi.pourOrigin.v1', raw)
}
