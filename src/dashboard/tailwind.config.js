/** @type {import('tailwindcss').Config} */
export default {
  content: [
    './index.html',
    './src/**/*.{ts,tsx}',
  ],
  theme: {
    extend: {
      colors: {
        background: '#050607',
        surface: '#0a0b0d',
        primary: {
          sky400: '#38bdf8',
          blue400: '#60a5fa',
          cyan400: '#22d3ee',
        },
        status: {
          good: '#34d399', // emerald-400
          warn: '#f59e0b', // amber-400
          bad: '#fb7185',  // rose-400
        },
      },
      fontFamily: {
        sans: ['Inter', 'ui-sans-serif', 'system-ui'],
        display: ['Space Grotesk', 'Inter', 'ui-sans-serif'],
      },
    },
  },
  plugins: [],
}













