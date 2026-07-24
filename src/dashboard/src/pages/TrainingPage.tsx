import { useMemo, useState } from 'react'
import SidebarLayout from './SidebarLayout'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'

export default function TrainingPage() {
  const [open, setOpen] = useState(false)
  const [powderName, setPowderName] = useState('boxA')
  const [cycleEndLimit, setCycleEndLimit] = useState('')
  const [targetWeight, setTargetWeight] = useState('250.0')
  const [episodes, setEpisodes] = useState('10')
  const [batchId, setBatchId] = useState('sim-007')
  const [startMsg, setStartMsg] = useState('')
  const ros = useRos()

  const startDisabled = useMemo(() => {
    if (!powderName.trim()) return true
    if (!targetWeight.trim()) return true
    if (!episodes.trim()) return true
    return false
  }, [powderName, targetWeight, episodes])

  const resetForm = () => {
    setPowderName('boxA')
    setCycleEndLimit('')
    setTargetWeight('250.0')
    setEpisodes('10')
    setBatchId('sim-007')
    setStartMsg('')
  }

  const startLightsOut = () => {
    setStartMsg('')
    if (!ros) {
      setStartMsg('ROS bridge not connected')
      return
    }
    const targetWeightG = Number.parseFloat(targetWeight)
    const episodeCount = Number.parseInt(episodes, 10)
    if (!Number.isFinite(targetWeightG) || targetWeightG <= 0) {
      setStartMsg('Target weight must be a number')
      return
    }
    if (!Number.isFinite(episodeCount) || episodeCount <= 0) {
      setStartMsg('Episodes must be a positive integer')
      return
    }
    const srv = new ROSLIB.Service({
      ros,
      name: '/bt_start_lightsout',
      serviceType: 'robot_common_msgs/srv/StartLightsOut',
    })
    const req = new ROSLIB.ServiceRequest({
      powder_name: powderName.trim(),
      cycle_end_limit: cycleEndLimit.trim(),
      target_weight_g: targetWeightG,
      episodes: episodeCount,
      batch_id: batchId.trim(),
    })
    srv.callService(
      req,
      (res: any) => {
        setStartMsg(res?.message || 'Lights-out training requested')
        if (res?.accepted) {
          setOpen(false)
        }
      },
      () => setStartMsg('Failed to call lights-out service')
    )
  }

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="flex items-end justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Training</h1>
            <p className="text-[var(--text-secondary)]">Configure and start lights-off training runs</p>
          </div>
        </div>

        <div className="grid grid-cols-12 gap-4">
          <div className="col-span-12 md:col-span-6">
            <GlassCard>
              <div className="flex flex-col gap-3">
                <h3 className="text-lg font-semibold">Lights-Off Training</h3>
                <p className="text-sm text-[var(--text-secondary)]">
                  Start a training cycle with metadata and operational limits.
                </p>
                <Button onClick={() => setOpen(true)}>Start Lights-Off Training</Button>
              </div>
            </GlassCard>
          </div>
        </div>
      </div>

      {open && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-[var(--overlay-backdrop)]">
          <div className="w-[92%] max-w-lg rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5 shadow-xl">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-lg font-semibold">Training Metadata</h2>
              <button
                className="text-[var(--text-muted)] hover:text-[var(--text-primary)]"
                onClick={() => setOpen(false)}
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            <div className="grid grid-cols-1 gap-3">
              <label className="text-sm text-[var(--text-secondary)]">
                Powder name
                <input
                  value={powderName}
                  onChange={(e) => setPowderName(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. Alumina 5um"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Cycle end limit
                <input
                  value={cycleEndLimit}
                  onChange={(e) => setCycleEndLimit(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 120s or 150 cycles"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Target weight (g)
                <input
                  value={targetWeight}
                  onChange={(e) => setTargetWeight(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 125.0"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Episodes
                <input
                  value={episodes}
                  onChange={(e) => setEpisodes(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 10"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Batch ID
                <input
                  value={batchId}
                  onChange={(e) => setBatchId(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. batch-2026-01-19"
                />
              </label>
            </div>

            {startMsg && (
              <p className="mt-3 text-xs text-[var(--text-secondary)]">{startMsg}</p>
            )}

            <div className="mt-5 flex items-center justify-end gap-2">
              <Button
                variant="ghost"
                onClick={() => {
                  resetForm()
                  setOpen(false)
                }}
              >
                Cancel
              </Button>
              <Button onClick={startLightsOut} disabled={startDisabled}>
                Start
              </Button>
            </div>
          </div>
        </div>
      )}
    </SidebarLayout>
  )
}







