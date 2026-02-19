// RosContext: provides a single shared ROSLIB.Ros connection to rosbridge
// so all pages/components reuse one websocket and share lifecycle.
import React, { createContext, useContext, useEffect, useState } from 'react'
import { ROSLIB } from './roslib'

const RosCtx = createContext<ROSLIB.Ros | null>(null)

export function RosProvider({ children }: { children: React.ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null)
  const url: string = (import.meta as any).env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'

  useEffect(() => {
    // Create a single ROS connection for the app
    const conn = new ROSLIB.Ros({ url })
    setRos(conn)
    // Attach error handler to avoid unhandled error events from roslib
    conn.on('error', () => {})
    return () => { try { conn.close() } catch {} }
  }, [url])

  return <RosCtx.Provider value={ros}>{children}</RosCtx.Provider>
}

export function useRos(): ROSLIB.Ros | null {
  return useContext(RosCtx)
}








