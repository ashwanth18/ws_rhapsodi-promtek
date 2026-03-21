import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useState,
} from 'react'

export type ThemePreference = 'light' | 'dark' | 'auto'
export type ResolvedTheme = 'light' | 'dark'

type ThemeContextValue = {
  themePreference: ThemePreference
  resolvedTheme: ResolvedTheme
  setThemePreference: (value: ThemePreference) => void
}

const STORAGE_KEY = 'rhapsodi.themePreference'
const ThemeContext = createContext<ThemeContextValue | null>(null)

function resolveTheme(preference: ThemePreference): ResolvedTheme {
  if (preference === 'light' || preference === 'dark') {
    return preference
  }
  return window.matchMedia('(prefers-color-scheme: dark)').matches
    ? 'dark'
    : 'light'
}

export function ThemeProvider({ children }: { children: React.ReactNode }) {
  const [themePreference, setThemePreferenceState] = useState<ThemePreference>(() => {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored === 'light' || stored === 'dark' || stored === 'auto') {
      return stored
    }
    return 'auto'
  })
  const [resolvedTheme, setResolvedTheme] = useState<ResolvedTheme>(() =>
    resolveTheme(themePreference)
  )

  useEffect(() => {
    const media = window.matchMedia('(prefers-color-scheme: dark)')

    const applyTheme = () => {
      const next = resolveTheme(themePreference)
      setResolvedTheme(next)
      document.documentElement.classList.remove('theme-light', 'theme-dark')
      document.documentElement.classList.add(
        next === 'dark' ? 'theme-dark' : 'theme-light'
      )
      document.documentElement.style.colorScheme = next
    }

    applyTheme()
    const onChange = () => {
      if (themePreference === 'auto') {
        applyTheme()
      }
    }
    media.addEventListener('change', onChange)
    return () => media.removeEventListener('change', onChange)
  }, [themePreference])

  const setThemePreference = (value: ThemePreference) => {
    setThemePreferenceState(value)
    localStorage.setItem(STORAGE_KEY, value)
  }

  const value = useMemo(
    () => ({ themePreference, resolvedTheme, setThemePreference }),
    [resolvedTheme, themePreference]
  )

  return <ThemeContext.Provider value={value}>{children}</ThemeContext.Provider>
}

export function useTheme(): ThemeContextValue {
  const ctx = useContext(ThemeContext)
  if (!ctx) {
    throw new Error('useTheme must be used within ThemeProvider')
  }
  return ctx
}
