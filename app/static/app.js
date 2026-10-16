/* BlueOS Self Test - wizard front end (Vue 2.7 + Vuetify 2.7, no build step).
 *
 * All API calls use relative URLs so the page works both on its own port and
 * through BlueOS's /extensionv2/<name>/ proxy (works_in_relative_paths).
 */
/* global Vue, Vuetify */

const api = {
  async get (path) {
    const r = await fetch(path, { cache: 'no-store' })
    if (!r.ok) throw new Error((await r.json().catch(() => ({}))).error || r.statusText)
    return r.json()
  },
  async post (path, body) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    })
    const data = await r.json().catch(() => ({}))
    if (!r.ok) throw new Error(data.error || r.statusText)
    return data
  }
}

// BlueOS core services (network test, file browser) live on the vehicle's
// main web port. When the page is opened directly on the extension port we
// still target port 80 on the same host.
function blueosRoot () {
  if (window.SELFTEST_SIM) return window.location.origin
  const p = window.location.port
  if (p && p !== '80' && p !== '443') return `${window.location.protocol}//${window.location.hostname}`
  return window.location.origin
}

const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms))

const TEST_DURATION_FALLBACK = 5

/* ------------------------------------------------------------------------ */
/* Network test: browser <-> vehicle, using BlueOS's built-in network-test   */
/* service (same endpoints as BlueOS's own speed test widget).               */
/* ------------------------------------------------------------------------ */
async function measureLatency (seconds, onUpdate) {
  const root = blueosRoot().replace(/^http/, 'ws')
  return new Promise((resolve) => {
    const rtts = []
    let sent = 0
    let ws
    let timer
    const finish = (error) => {
      clearInterval(timer)
      try { ws.close() } catch (e) { /* ignore */ }
      if (!rtts.length) return resolve({ error: error || 'No latency replies', sent })
      const avg = rtts.reduce((a, b) => a + b, 0) / rtts.length
      const jitter = rtts.length > 1
        ? rtts.slice(1).reduce((a, v, i) => a + Math.abs(v - rtts[i]), 0) / (rtts.length - 1)
        : 0
      resolve({
        min: +Math.min(...rtts).toFixed(2),
        avg: +avg.toFixed(2),
        max: +Math.max(...rtts).toFixed(2),
        jitter: +jitter.toFixed(2),
        sent,
        lost: Math.max(0, sent - rtts.length)
      })
    }
    try {
      ws = new WebSocket(`${root}/network-test/ws`)
    } catch (e) {
      return resolve({ error: String(e) })
    }
    ws.onmessage = (ev) => {
      const rtt = performance.now() - parseFloat(ev.data)
      if (isFinite(rtt)) { rtts.push(rtt); onUpdate && onUpdate(rtt) }
    }
    ws.onerror = () => finish('WebSocket error (is BlueOS network-test service available?)')
    ws.onopen = () => {
      const t0 = performance.now()
      timer = setInterval(() => {
        if (performance.now() - t0 > seconds * 1000) return finish()
        sent++
        ws.send(performance.now().toString())
      }, 100)
    }
    setTimeout(() => finish('Timed out'), (seconds + 4) * 1000)
  })
}

async function measureDownload (seconds, onUpdate) {
  const controller = new AbortController()
  const t0 = performance.now()
  let bytes = 0
  const stop = setTimeout(() => controller.abort(), seconds * 1000)
  try {
    const r = await fetch(`${blueosRoot()}/network-test/get_file?size=${512 * 2 ** 20}`,
      { signal: controller.signal, cache: 'no-store' })
    if (!r.ok) throw new Error(`HTTP ${r.status}`)
    const reader = r.body.getReader()
    for (;;) {
      const { done, value } = await reader.read()
      if (done) break
      bytes += value.length
      const dt = (performance.now() - t0) / 1000
      onUpdate && onUpdate(8 * bytes / dt / 1e6)
    }
  } catch (e) {
    if (e.name !== 'AbortError') { clearTimeout(stop); return { error: String(e.message || e) } }
  }
  clearTimeout(stop)
  const dt = (performance.now() - t0) / 1000
  return { mbps: +(8 * bytes / dt / 1e6).toFixed(1), bytes }
}

function measureUpload (seconds, onUpdate) {
  return new Promise((resolve) => {
    const payload = new Uint8Array(256 * 2 ** 20)
    const xhr = new XMLHttpRequest()
    let t0 = 0
    let loaded = 0
    const result = () => {
      const dt = (performance.now() - t0) / 1000
      return { mbps: +(8 * loaded / dt / 1e6).toFixed(1), bytes: loaded }
    }
    xhr.upload.onprogress = (ev) => {
      loaded = ev.loaded
      onUpdate && onUpdate(8 * loaded / ((performance.now() - t0) / 1000) / 1e6)
    }
    xhr.onload = () => { clearTimeout(stop); resolve(result()) }
    xhr.onerror = () => { clearTimeout(stop); resolve({ error: 'Upload failed' }) }
    xhr.onabort = () => resolve(result())
    const stop = setTimeout(() => xhr.abort(), seconds * 1000)
    xhr.open('POST', `${blueosRoot()}/network-test/post_file`)
    t0 = performance.now()
    xhr.send(payload)
  })
}

function networkVerdict (res) {
  if (res.error || !res.latency || res.latency.error) return 'FAIL'
  const down = res.download_mbps || 0
  const up = res.upload_mbps || 0
  // A tether link only needs a few Mbit/s for video + telemetry
  if (down < 5 || up < 5) return 'WARN'
  return 'PASS'
}

/* ------------------------------------------------------------------------ */

Vue.component('result-chip', {
  props: ['result'],
  computed: {
    color () {
      return {
        PASS: 'success', FAIL: 'error', ERROR: 'error', ABORTED: 'error',
        WARN: 'warning', UPDATE: 'warning', SKIPPED: 'grey', DONE: 'primary'
      }[this.result] || 'grey lighten-1'
    }
  },
  template: '<v-chip small label :color="color" text-color="white" class="font-weight-medium">{{ result || "—" }}</v-chip>'
})

// Thruster positions, top-down with the bow up. Numbering follows ArduSub
// (libraries/AP_Motors/AP_Motors6DOF.cpp) and the BlueOS vehicle models:
// 1 front-right, 2 front-left, 3 rear-right, 4 rear-left (vectored);
// standard verticals 5 right / 6 left; Heavy verticals 5 FR, 6 FL, 7 RR, 8 RL.
const ROV_HORIZONTAL = [
  { n: 1, x: 212, y: 84, a: 45 }, { n: 2, x: 108, y: 84, a: -45 },
  { n: 3, x: 212, y: 296, a: -45 }, { n: 4, x: 108, y: 296, a: 45 }
]
const ROV_VERTICAL = {
  rov: [{ n: 5, x: 266, y: 196 }, { n: 6, x: 54, y: 196 }],
  heavy: [{ n: 5, x: 270, y: 116 }, { n: 6, x: 50, y: 116 }, { n: 7, x: 270, y: 264 }, { n: 8, x: 50, y: 264 }]
}

Vue.component('frame-diagram', {
  // kind: 'rov' | 'heavy' | 'boat'; active/failed: motor numbers (ROV) or 'left'/'right' (boat)
  props: { kind: String, active: { type: Array, default: () => [] }, failed: { type: Array, default: () => [] } },
  computed: {
    horizontal () { return ROV_HORIZONTAL },
    vertical () { return ROV_VERTICAL[this.kind] || [] }
  },
  methods: {
    cls (key) {
      return { active: this.active.includes(key), failed: this.failed.includes(key) }
    }
  },
  template: '#frame-diagram-template'
})

Vue.component('blueboat-rotation', {
  props: ['side'],   // 'left' | 'right' | null
  template: '#blueboat-rotation-template'
})

new Vue({
  el: '#app',
  template: '#app-template',
  vuetify: new Vuetify({
    theme: {
      themes: {
        light: {
          primary: '#135ea4',     // Mariner Blue (BlueOS Blue)
          secondary: '#2699d0',   // BR Blue
          accent: '#fedd00',      // Tether Yellow
          success: '#7ac943',
          error: '#ff1d25',
          warning: '#ff6d2d',
          info: '#2699d0'
        }
      }
    }
  }),
  data: () => ({
    step: 1,
    status: null,
    telemetry: {},
    detectPolling: null,
    form: { operator: '', serial: '', notes: '' },
    safetyConfirmed: false,
    sessionError: null,
    sessionStarting: false,
    // network
    net: { running: false, phase: '', live: null, result: null },
    // devices / sonar
    devices: { running: false, result: null, error: null },
    // power
    power: { measured_v: '', measured_a: '', preview: null, confirmWarnings: false, busy: false, done: null, error: null },
    // tests
    testIndex: 0,
    testState: null,
    testError: null,
    autoAdvance: true,
    advanceCountdown: null,
    paused: false,
    pollTimer: null,
    now: Date.now(),
    phaseSince: Date.now(),
    telemetryTimer: null,
    // end-of-sequence thruster review
    review: { open: false, done: false, selected: [], submitted: false, failed: [], busy: false },
    // finish
    finish: null,
    finishing: false
  }),
  computed: {
    vehicle () { return this.status && this.status.vehicle },
    tests () { return (this.vehicle && this.vehicle.tests) || [] },
    currentTest () { return this.tests[this.testIndex] || null },
    results () { return (this.status && this.status.results) || {} },
    isBoat () { return this.vehicle && this.vehicle.model_key === 'blueboat' },
    detecting () { return this.status && this.status.detect.status === 'running' },
    steps () {
      return [
        { n: 1, title: 'Vehicle' },
        { n: 2, title: 'Network' },
        { n: 3, title: 'Devices' },
        { n: 4, title: 'Power sensor' },
        { n: 5, title: 'Hardware tests' },
        { n: 6, title: 'Finish' }
      ]
    },
    testPhase () { return this.testState ? this.testState.phase : 'idle' },
    waitingToStart () {
      return ['preparing', 'rebooting', 'cooldown'].includes(this.testPhase) && !(this.testState && this.testState.sweep_result)
    },
    tiltNeedsReboot () { return !!(this.currentTest && this.currentTest.category === 'tilt' && this.currentTest.stabilized) },
    tiltStages () {
      return [
        { n: 1, label: 'Turn camera stabilization off and reboot the autopilot' },
        { n: 2, label: 'Test the tilt range of motion (watch the camera)' },
        { n: 3, label: 'Turn stabilization back on and reboot the autopilot' }
      ]
    },
    tiltStage () {
      if (!this.testState) return 0
      if (this.testState.sweep_result) return this.testPhase === 'done' ? 4 : 3
      return this.testPhase === 'running' ? 2 : 1
    },
    phaseElapsed () { return Math.max(0, Math.round((this.now - this.phaseSince) / 1000)) },
    running () { return this.testPhase === 'running' },
    busyTest () { return ['preparing', 'running', 'rebooting', 'cooldown', 'restoring'].includes(this.testPhase) },
    testProgress () {
      if (!this.testState || !this.testState.duration) return 0
      return Math.min(100, 100 * this.testState.elapsed / this.testState.duration)
    },
    lastResult () {
      return this.currentTest ? this.results[this.currentTest.id] : null
    },
    showOutcome () {
      return this.testPhase === 'done' && this.testState && this.testState.test_id === (this.currentTest && this.currentTest.id)
    },
    troubleshootingUrl () {
      const map = (this.status && this.status.troubleshooting) || {}
      return this.currentTest ? map[this.currentTest.category] : ''
    },
    boatSide () {
      if (!this.isBoat || !this.currentTest || this.currentTest.category !== 'motor') return null
      return { 73: 'left', 74: 'right' }[this.currentTest.function] || null
    },
    versions () { return this.status && this.status.versions },
    frameKind () {
      if (!this.vehicle) return null
      if (this.isBoat) return 'boat'
      if (this.vehicle.firmware === 'ArduSub' && [1, 2].includes(Number(this.vehicle.frame))) {
        return Number(this.vehicle.frame) === 2 ? 'heavy' : 'rov'
      }
      return null
    },
    motorTests () { return this.tests.filter(t => t.category === 'motor') },
    motorFailedKeys () {
      return this.motorTests
        .filter(t => this.review.open ? this.review.selected.includes(t.id) : (this.results[t.id] || {}).result === 'FAIL')
        .map(this.motorKey)
    },
    motorTroubleshooting () {
      const tips = [
        'With power off, spin the propeller by hand. It should turn freely with no grinding; clear any debris or hair.',
        'Check the thruster\'s ESC signal cable is plugged into the correct autopilot output, with signal and ground the right way round.',
        'Listen for the ESC start-up beeps when the battery is connected; no beeps means the ESC has no power (check the power distribution and connectors).',
        'Check the three thruster wires to the ESC (or penetrator connection) for damage or loose joints.',
        'Swap the signal cable with a working thruster\'s output: if the fault moves, it is the autopilot output or cable; if it stays, it is the ESC or thruster.'
      ]
      if (this.isBoat) {
        tips.splice(3, 0, 'If it spins the wrong way, swap any two of the three thruster wires and check the correct-handed propeller is fitted.')
      }
      return tips
    },
    restorePending () {
      const p = (this.status && this.status.restore_pending) || {}
      return Object.keys(p)
    },
    summaryRows () {
      const rows = []
      const r = this.results
      if (r.versions) rows.push({ name: 'Software versions', result: r.versions.result, detail: r.versions.summary || '' })
      if (r.network) rows.push({ name: 'Network (browser ↔ vehicle)', result: r.network.result, detail: this.netDetail(r.network) })
      if (r.devices) rows.push({ name: 'Network devices & sonars', result: r.devices.result, detail: r.devices.summary || '' })
      if (r.power) rows.push({ name: 'Power sensor calibration', result: r.power.result, detail: this.powerDetail(r.power) })
      for (const t of this.tests) {
        const res = r[t.id]
        rows.push({
          name: `${t.name} (output ${t.channel})`,
          result: res ? res.result : null,
          detail: res ? (res.error || (res.fail_at_s != null ? `flagged at ${res.fail_at_s.toFixed(1)} s` : '')) : 'not run'
        })
      }
      return rows
    },
    fileBrowserUrl () { return `${blueosRoot()}/file-browser/files/extensions/selftest/` },
    blueosFileBrowserUrl () { return `${blueosRoot()}/tools/file-browser` }
  },
  watch: {
    step (n, old) {
      if (old === 5) this.stopPolling()
      if (n === 5) this.startPolling()
    }
  },
  async mounted () {
    window.addEventListener('keydown', this.onKey)
    await this.refresh()
    if (this.status && this.status.connected && !this.vehicle && !this.detecting) this.detect()
    else if (this.status && !this.status.connected) this.waitForConnection()
    this.telemetryTimer = setInterval(this.refreshTelemetry, 1000)
  },
  beforeDestroy () {
    window.removeEventListener('keydown', this.onKey)
    this.stopPolling()
    clearInterval(this.telemetryTimer)
  },
  methods: {
    async refresh () {
      try {
        this.status = await api.get('api/status')
        window.SELFTEST_SIM = !!this.status.sim
      } catch (e) { /* keep last */ }
    },
    async refreshTelemetry () {
      if (this.step !== 4 && this.step !== 1) return
      try { this.telemetry = await api.get('api/telemetry') } catch (e) { /* ignore */ }
    },
    async waitForConnection () {
      while (this.status && !this.status.connected) {
        await sleep(2000)
        await this.refresh()
      }
      if (!this.vehicle) this.detect()
    },
    async detect () {
      try { await api.post('api/detect') } catch (e) { return }
      clearInterval(this.detectPolling)
      this.detectPolling = setInterval(async () => {
        await this.refresh()
        if (!this.detecting) clearInterval(this.detectPolling)
      }, 400)
    },
    async startSession () {
      this.sessionStarting = true
      this.sessionError = null
      try {
        await api.post('api/session/start', this.form)
        this.testIndex = 0
        this.review = { open: false, done: false, selected: [], submitted: false, failed: [], busy: false }
        this.finish = null
        this.net.result = null
        this.devices.result = null
        this.power.done = null
        this.power.preview = null
        await this.refresh()
        this.step = 2
      } catch (e) {
        this.sessionError = e.message
      } finally {
        this.sessionStarting = false
      }
    },

    /* ---- network ---- */
    async runNetwork () {
      this.net.running = true
      this.net.result = null
      const res = {}
      this.net.phase = 'Latency'
      res.latency = await measureLatency(5, (v) => { this.net.live = `${v.toFixed(1)} ms` })
      this.net.phase = 'Download'
      const down = await measureDownload(5, (v) => { this.net.live = `${v.toFixed(1)} Mbps` })
      this.net.phase = 'Upload'
      const up = await measureUpload(5, (v) => { this.net.live = `${v.toFixed(1)} Mbps` })
      res.download_mbps = down.mbps
      res.upload_mbps = up.mbps
      const errors = [res.latency.error, down.error, up.error].filter(Boolean)
      if (errors.length) res.error = errors.join('; ')
      res.verdict = networkVerdict(res)
      this.net.result = res
      this.net.running = false
      this.net.phase = ''
      this.net.live = null
      try { await api.post('api/network', res) } catch (e) { /* logged server side if possible */ }
      await this.refresh()
    },
    dash (v) { return v === null || v === undefined ? '—' : v },
    netDetail (n) {
      if (!n) return ''
      const lat = n.latency || {}
      return `${lat.avg != null ? lat.avg + ' ms avg' : ''}  ↓ ${this.dash(n.download_mbps)} Mbps  ↑ ${this.dash(n.upload_mbps)} Mbps${n.error ? ' — ' + n.error : ''}`
    },

    /* ---- devices / sonar ---- */
    async runDevices () {
      this.devices.running = true
      this.devices.error = null
      try {
        this.devices.result = await api.post('api/devices/scan')
      } catch (e) {
        this.devices.error = e.message
      } finally {
        this.devices.running = false
        await this.refresh()
      }
    },

    /* ---- power ---- */
    async previewPower () {
      this.power.busy = true
      this.power.error = null
      this.power.confirmWarnings = false
      try {
        this.power.preview = await api.post('api/calibration/preview', {
          measured_v: this.power.measured_v, measured_a: this.power.measured_a
        })
      } catch (e) { this.power.error = e.message } finally { this.power.busy = false }
    },
    async applyPower () {
      this.power.busy = true
      try {
        this.power.done = await api.post('api/calibration/apply', {
          measured_v: this.power.measured_v,
          measured_a: this.power.measured_a,
          confirmed: this.power.confirmWarnings
        })
        await this.refresh()
      } catch (e) { this.power.error = e.message } finally { this.power.busy = false }
    },
    async skipPower () {
      await api.post('api/calibration/skip')
      await this.refresh()
      this.step = 5
    },
    powerDetail (p) {
      if (!p) return ''
      if (p.skipped) return `skipped (reported ${this.dash(p.reported_v)} V)`
      const parts = Object.entries(p.changes || {}).map(([k, v]) => `${k} ${v.old} → ${v.new}`)
      return parts.join(', ') + (p.warnings && p.warnings.length ? ` — ${p.warnings.length} warning(s)` : '')
    },

    /* ---- hardware tests ---- */
    startPolling () {
      this.stopPolling()
      const tick = async () => {
        try {
          const s = await api.get('api/test/state')
          const prevPhase = this.testPhase
          this.now = Date.now()
          if (s.phase !== prevPhase) this.phaseSince = this.now
          this.testState = s
          if (s.restore_pending && this.status) this.status.restore_pending = s.restore_pending
          if (prevPhase !== 'done' && s.phase === 'done') this.onTestDone()
        } catch (e) { /* server busy */ }
      }
      tick()
      this.pollTimer = setInterval(tick, 250)
    },
    stopPolling () {
      clearInterval(this.pollTimer)
      this.pollTimer = null
      this.cancelAdvance()
    },
    async startTest () {
      if (!this.currentTest) return
      this.cancelAdvance()
      this.testError = null
      try {
        await api.post('api/test/start', { id: this.currentTest.id })
        this.testState = { phase: 'preparing', message: 'Preparing', elapsed: 0, duration: TEST_DURATION_FALLBACK, test_id: this.currentTest.id }
      } catch (e) { this.testError = e.message }
    },
    async onTestDone () {
      await this.refresh()
      const res = this.lastResult
      if (res && res.result === 'PASS' && this.autoAdvance && !this.paused) {
        this.scheduleAdvance()
      }
    },
    scheduleAdvance () {
      this.cancelAdvance()
      this.advanceCountdown = 3
      this._advanceTimer = setInterval(() => {
        this.advanceCountdown--
        if (this.advanceCountdown <= 0) {
          this.cancelAdvance()
          this.nextTest(true)
        }
      }, 1000)
    },
    cancelAdvance () {
      clearInterval(this._advanceTimer)
      this.advanceCountdown = null
    },
    blueosPage (path) { return `${blueosRoot()}${path}` },
    testDuration (t) {
      const st = (this.status && this.status.settings) || {}
      const key = { motor: 'motor_duration_s', actuator: 'actuator_duration_s' }[t.category] || 'duration_s'
      return st[key] || 5
    },
    motorKey (t) {
      if (this.isBoat) return { 73: 'left', 74: 'right' }[t.function] || t.name
      return t.motor_number
    },
    openReview () {
      this.review = {
        open: true,
        done: false,
        submitted: false,
        busy: false,
        failed: [],
        selected: this.motorTests.filter(t => (this.results[t.id] || {}).result === 'FAIL').map(t => t.id)
      }
    },
    toggleReview (id) {
      if (this.review.submitted) return
      const i = this.review.selected.indexOf(id)
      if (i >= 0) this.review.selected.splice(i, 1)
      else this.review.selected.push(id)
    },
    async submitReview () {
      this.review.busy = true
      try {
        const res = await api.post('api/motors/review', { failed_ids: this.review.selected })
        this.review.failed = res.failed
        this.review.submitted = true
        await this.refresh()
        if (!res.failed.length) this.continueAfterReview()
      } catch (e) { this.testError = e.message } finally { this.review.busy = false }
    },
    continueAfterReview () {
      this.review.open = false
      this.review.done = true
      this.nextTest(false)
    },
    async nextTest (autostart) {
      this.cancelAdvance()
      const cur = this.currentTest
      const next = this.tests[this.testIndex + 1]
      if (cur && cur.category === 'motor' && (!next || next.category !== 'motor') && !this.review.done) {
        // end of the thruster sequence: stop the motors, then give the tester
        // a second chance to report thrusters that didn't work
        try { await api.post('api/motors/end') } catch (e) { /* ignore */ }
        this.openReview()
        return
      }
      if (this.testIndex < this.tests.length - 1) {
        const wasMotor = this.currentTest && this.currentTest.category === 'motor'
        this.testIndex++
        this.testState = null
        if (wasMotor && this.currentTest.category !== 'motor') {
          try { await api.post('api/motors/end') } catch (e) { /* ignore */ }
        }
        if (autostart && this.autoAdvance && !this.paused) this.startTest()
      } else {
        try { await api.post('api/motors/end') } catch (e) { /* ignore */ }
        this.step = 6
        this.finishSession()
      }
    },
    async skipTest () {
      if (this.busyTest) return
      await api.post('api/test/skip', { id: this.currentTest.id })
      await this.refresh()
      this.nextTest(false)
    },
    selectTest (i) {
      if (this.busyTest) return
      this.cancelAdvance()
      this.review.open = false
      if (this.tests[i] && this.tests[i].category === 'motor') this.review.done = false
      this.testIndex = i
      this.testState = null
    },
    async flagFail () {
      if (!this.running) return
      try { await api.post('api/test/fail') } catch (e) { /* ignore */ }
    },
    async abortTest () {
      this.cancelAdvance()
      try { await api.post('api/test/abort') } catch (e) { /* ignore */ }
    },
    async restoreAll () {
      try { await api.post('api/restore') } catch (e) { this.testError = e.message }
      await this.refresh()
    },
    onKey (e) {
      if (e.code !== 'Space' && e.key !== ' ') return
      const tag = (e.target && e.target.tagName) || ''
      if (['INPUT', 'TEXTAREA'].includes(tag)) return
      if (this.step === 5 && !this.review.open) {
        e.preventDefault()
        if (this.running) this.flagFail()
      }
    },
    expectText (t) {
      return t ? t.expect : ''
    },

    /* ---- finish ---- */
    async finishSession () {
      this.finishing = true
      try {
        this.finish = await api.post('api/session/finish')
      } catch (e) {
        this.finish = { error: e.message }
      } finally {
        this.finishing = false
        await this.refresh()
      }
    },
    restart () {
      this.step = 1
      this.finish = null
      this.testIndex = 0
      this.testState = null
      this.safetyConfirmed = false
      this.detect()
    }
  }
})
