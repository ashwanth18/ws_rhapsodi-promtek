import { useEffect, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'

const RUN_STATE_TOPIC =
  (import.meta as any).env.VITE_RUN_STATE_TOPIC || '/orchestrator/run_state'
const WEIGHT_TOPIC = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'

export function useShellTelemetry() {
  const ros = useRos()
  const [runState, setRunState] = useState('idle')
  const [weightStale, setWeightStale] = useState(true)
  const [lastWeightTs, setLastWeightTs] = useState<number | null>(null)

  useEffect(() => {
    if (!ros) return
    const runStateTopic = new ROSLIB.Topic({
      ros,
      name: RUN_STATE_TOPIC,
      messageType: 'std_msgs/String',
    })
    runStateTopic.subscribe((msg: { data: string }) => {
      setRunState((msg.data || 'idle').toLowerCase())
    })
    const weightTopic = new ROSLIB.Topic({
      ros,
      name: WEIGHT_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    weightTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setLastWeightTs(Date.now())
        setWeightStale(false)
      }
    })
    return () => {
      runStateTopic.unsubscribe()
      weightTopic.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    const id = window.setInterval(() => {
      setWeightStale(!lastWeightTs || Date.now() - lastWeightTs > 1500)
    }, 500)
    return () => window.clearInterval(id)
  }, [lastWeightTs])

  return { runState, weightStale }
}
