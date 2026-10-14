import { useCallback, useEffect, useState } from 'react'
import { api } from '../api/client'
import type { Preset } from '../types'

export function usePresets() {
  const [presets, setPresets] = useState<Preset[]>([])
  const [error, setError] = useState<string>('')

  const refresh = useCallback(async () => {
    try {
      const res = await api.presets()
      if (res.ok && Array.isArray(res.data)) setPresets(res.data)
    } catch (e) {
      setError(String(e))
    }
  }, [])

  useEffect(() => {
    void refresh()
  }, [refresh])

  return { presets, error, refresh }
}
