/**
 * Tankbot 2025 Mobile Controller
 * Low-latency touch joystick, 6-DOF arm sliders, radar HUD, and telemetry client.
 */

class TankbotClient {
  constructor() {
    this.ws = null;
    this.connected = false;
    this.isDraggingJoystick = false;
    this.maxSpeedPercent = 80;
    this.isClawOpen = false;
    this.activeSlider = null;
    this.lastJoystickSent = 0;

    this.initElements();
    this.initTabs();
    this.initJoystick();
    this.initDPad();
    this.initArmControls();
    this.initModes();
    this.initKeyboard();
    this.connectWebSocket();

    // Start keep-alive ping loop
    setInterval(() => this.sendPing(), 1500);
  }

  haptic(ms = 20) {
    if ('vibrate' in navigator) {
      try { navigator.vibrate(ms); } catch (e) {}
    }
  }

  initElements() {
    this.connBadge = document.getElementById('conn-badge');
    this.hwBadge = document.getElementById('hw-badge');
    this.btnEstop = document.getElementById('btn-estop');
    this.valDistance = document.getElementById('val-distance');
    this.proximityAlert = document.getElementById('proximity-alert');
    this.speedSlider = document.getElementById('speed-slider');
    this.speedLimitLabel = document.getElementById('speed-limit-label');
    this.leftSpeedVal = document.getElementById('left-speed-val');
    this.rightSpeedVal = document.getElementById('right-speed-val');
    this.curModeLabel = document.getElementById('cur-mode-label');
    this.modeExplainText = document.getElementById('mode-explain-text');
    this.seqStatus = document.getElementById('seq-status');

    this.imuPitch = document.getElementById('imu-pitch');
    this.imuRoll = document.getElementById('imu-roll');
    this.imuStatus = document.getElementById('imu-status');
    this.batVoltage = document.getElementById('bat-voltage');

    this.irSensors = [
      document.getElementById('ir-0'),
      document.getElementById('ir-1'),
      document.getElementById('ir-2'),
      document.getElementById('ir-3')
    ];

    // Speed Slider
    this.speedSlider.addEventListener('input', (e) => {
      this.maxSpeedPercent = parseInt(e.target.value);
      this.speedLimitLabel.textContent = `${this.maxSpeedPercent}%`;
    });

    // Emergency Stop
    this.btnEstop.addEventListener('click', () => {
      this.haptic([100, 50, 150]);
      this.send({ type: 'emergency_stop' });
    });

    document.getElementById('btn-reset-estop')?.addEventListener('click', () => {
      this.haptic();
      this.send({ type: 'reset_stop' });
    });

    document.getElementById('btn-reconnect')?.addEventListener('click', () => {
      this.connectWebSocket();
    });
  }

  /* TABS */
  initTabs() {
    const hash = window.location.hash.replace('#', '');
    if (hash) {
      setTimeout(() => {
        const item = document.querySelector('.nav-item[data-tab="' + hash + '"]');
        if (item) item.click();
      }, 50);
    }
    const navItems = document.querySelectorAll('.nav-item');
    const panels = document.querySelectorAll('.tab-panel');

    navItems.forEach(item => {
      item.addEventListener('click', () => {
        this.haptic(15);
        const tabId = item.dataset.tab;

        navItems.forEach(n => n.classList.remove('active'));
        panels.forEach(p => p.classList.remove('active'));

        item.classList.add('active');
        document.getElementById(tabId)?.classList.add('active');
      });
    });
  }

  /* WEBSOCKET */
  connectWebSocket() {
    if (this.ws) {
      this.ws.close();
    }

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    this.connBadge.textContent = 'CONNECTING';
    this.connBadge.className = 'badge badge-sim';

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      this.connected = true;
      this.connBadge.textContent = 'ONLINE';
      this.connBadge.className = 'badge badge-connected';
      console.log('Connected to Tankbot WebSocket server.');
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry') {
          this.renderTelemetry(msg.data);
        } else if (msg.type === 'init') {
          this.hwBadge.textContent = msg.config?.is_simulation ? 'SIMULATION' : 'PI GPIO';
          this.hwBadge.className = msg.config?.is_simulation ? 'badge badge-sim' : 'badge badge-real';
          this.renderTelemetry(msg.telemetry);
        }
      } catch (e) {
        console.error('Error parsing WS message:', e);
      }
    };

    this.ws.onclose = () => {
      this.connected = false;
      this.connBadge.textContent = 'OFFLINE';
      this.connBadge.className = 'badge badge-disconnected';
      setTimeout(() => this.connectWebSocket(), 2000);
    };

    this.ws.onerror = (err) => {
      console.warn('WS error:', err);
      this.ws.close();
    };
  }

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }

  sendPing() {
    if (this.connected) {
      this.send({ type: 'ping' });
    }
  }

  /* VIRTUAL JOYSTICK */
  initJoystick() {
    const zone = document.getElementById('joystick-zone');
    const stick = document.getElementById('joystick-stick');
    const base = zone.querySelector('.joystick-base');

    const maxRadius = 45; // Max displacement in px

    const handleStart = (e) => {
      e.preventDefault();
      this.isDraggingJoystick = true;
      handleMove(e);
    };

    const handleMove = (e) => {
      if (!this.isDraggingJoystick) return;
      e.preventDefault();

      const rect = base.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;

      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;

      let dx = clientX - centerX;
      let dy = clientY - centerY;

      const dist = Math.hypot(dx, dy);
      if (dist > maxRadius) {
        const angle = Math.atan2(dy, dx);
        dx = Math.cos(angle) * maxRadius;
        dy = Math.sin(angle) * maxRadius;
      }

      stick.style.transform = `translate(${dx}px, ${dy}px)`;

      // Normalization: throttle (-1 to +1, up is positive), steering (-1 to +1, right is positive)
      const steering = (dx / maxRadius);
      const throttle = -(dy / maxRadius); // Inverted so pulling forward is positive

      const now = performance.now();
      if (now - this.lastJoystickSent > 40) { // Max ~25Hz transmission
        this.send({
          type: 'joystick',
          throttle: throttle * (this.maxSpeedPercent / 100.0),
          steering: steering * (this.maxSpeedPercent / 100.0)
        });
        this.lastJoystickSent = now;
      }
    };

    const handleEnd = (e) => {
      if (!this.isDraggingJoystick) return;
      e.preventDefault();
      this.isDraggingJoystick = false;
      stick.style.transform = `translate(0px, 0px)`;
      this.send({ type: 'joystick', throttle: 0, steering: 0 });
    };

    zone.addEventListener('pointerdown', handleStart);
    window.addEventListener('pointermove', handleMove);
    window.addEventListener('pointerup', handleEnd);
    window.addEventListener('pointercancel', handleEnd);
  }

  /* D-PAD */
  initDPad() {
    const buttons = document.querySelectorAll('.dpad-btn');
    buttons.forEach(btn => {
      const dir = btn.dataset.dir;

      const start = (e) => {
        e.preventDefault();
        this.haptic(15);
        this.send({
          type: 'dpad',
          direction: dir,
          speed: this.maxSpeedPercent
        });
      };

      const end = (e) => {
        e.preventDefault();
        if (dir !== 'stop') {
          this.send({ type: 'dpad', direction: 'stop' });
        }
      };

      btn.addEventListener('pointerdown', start);
      btn.addEventListener('pointerup', end);
      btn.addEventListener('pointerleave', end);
    });
  }

  /* ROBOTIC ARM CONTROLS */
  initArmControls() {
    // Preset Action Buttons
    const presetButtons = document.querySelectorAll('.preset-btn');
    presetButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        this.haptic(25);
        const preset = btn.dataset.preset;
        this.send({ type: 'preset', name: preset });
      });
    });

    // Joint Sliders
    const sliders = document.querySelectorAll('.servo-slider');
    sliders.forEach(slider => {
      const servoId = parseInt(slider.dataset.servo);
      const valDisplay = document.getElementById(`val-servo-servoId`);

      slider.addEventListener('pointerdown', () => { this.activeSlider = servoId; });
      slider.addEventListener('pointerup', () => { this.activeSlider = null; });

      slider.addEventListener('input', (e) => {
        const pos = parseInt(e.target.value);
        const readout = document.getElementById(`val-servo-${servoId}`);
        if (readout) readout.textContent = pos;

        this.send({
          type: 'servo',
          id: servoId,
          pos: pos,
          duration: 50
        });
      });
    });

    // Step Buttons (+ / -)
    const stepBtns = document.querySelectorAll('.btn-step');
    stepBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        this.haptic(15);
        const servoId = parseInt(btn.dataset.servo);
        const step = parseInt(btn.dataset.step);
        const slider = document.querySelector(`.servo-slider[data-servo="${servoId}"]`);
        if (!slider) return;

        let newVal = parseInt(slider.value) + step;
        newVal = Math.max(parseInt(slider.min), Math.min(parseInt(slider.max), newVal));
        slider.value = newVal;

        const readout = document.getElementById(`val-servo-${servoId}`);
        if (readout) readout.textContent = newVal;

        this.send({
          type: 'servo',
          id: servoId,
          pos: newVal,
          duration: 60
        });
      });
    });
  }

  /* MODES */
  initModes() {
    const modeBtns = document.querySelectorAll('.mode-btn');
    const modeDescriptions = {
      'MANUAL': 'Manual remote control via touch joystick and arm sliders.',
      'OBSTACLE_AVOIDANCE': 'Automatic ultrasonic roaming. Reverses/turns when obstacle detected within 29 cm.',
      'OBJECT_FOLLOW': 'Ultrasonic following. Robot tracks an object and maintains a distance of 20-35 cm.',
      'LINE_FOLLOW': 'Differential infrared line tracking navigation.'
    };

    modeBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        this.haptic(20);
        const mode = btn.dataset.mode;
        modeBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        this.curModeLabel.textContent = mode;
        this.modeExplainText.textContent = modeDescriptions[mode] || '';

        this.send({ type: 'mode', mode: mode });
      });
    });
  }

  /* DESKTOP KEYBOARD CONTROLS */
  initKeyboard() {
    const activeKeys = new Set();

    const handleKeys = () => {
      if (this.curModeLabel.textContent !== 'MANUAL') return;

      let throttle = 0;
      let steering = 0;

      if (activeKeys.has('ArrowUp') || activeKeys.has('w')) throttle += 1;
      if (activeKeys.has('ArrowDown') || activeKeys.has('s')) throttle -= 1;
      if (activeKeys.has('ArrowLeft') || activeKeys.has('a')) steering -= 1;
      if (activeKeys.has('ArrowRight') || activeKeys.has('d')) steering += 1;

      this.send({
        type: 'joystick',
        throttle: throttle * (this.maxSpeedPercent / 100.0),
        steering: steering * (this.maxSpeedPercent / 100.0)
      });
    };

    window.addEventListener('keydown', (e) => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', ' '].includes(e.key)) {
        if (e.key === ' ') {
          this.send({ type: 'emergency_stop' });
          return;
        }
        activeKeys.add(e.key);
        handleKeys();
      }
    });

    window.addEventListener('keyup', (e) => {
      if (activeKeys.has(e.key)) {
        activeKeys.delete(e.key);
        handleKeys();
      }
    });
  }

  /* TELEMETRY RENDERING */
  renderTelemetry(data) {
    if (!data) return;

    // Distance & Radar
    const distCm = data.distance_cm !== undefined ? data.distance_cm : (data.distance_mm / 10.0);
    this.valDistance.textContent = distCm.toFixed(1);

    if (distCm < 20) {
      this.proximityAlert.textContent = 'COLLISION WARNING';
      this.proximityAlert.className = 'proximity-pill alert';
    } else if (distCm < 35) {
      this.proximityAlert.textContent = 'PROXIMITY CAUTION';
      this.proximityAlert.className = 'proximity-pill warn';
    } else {
      this.proximityAlert.textContent = 'PATH CLEAR';
      this.proximityAlert.className = 'proximity-pill safe';
    }

    // Speeds
    this.leftSpeedVal.textContent = `${data.left_speed}%`;
    this.rightSpeedVal.textContent = `${data.right_speed}%`;

    // Active Sequence
    if (data.active_sequence) {
      this.seqStatus.textContent = `EXECUTING: ${data.active_sequence.toUpperCase()}`;
    } else {
      this.seqStatus.textContent = 'IDLE';
    }

    // Servos (update sliders if not currently touched by user)
    if (data.servo_positions) {
      for (const [sidStr, pos] of Object.entries(data.servo_positions)) {
        const sid = parseInt(sidStr);
        const readout = document.getElementById(`val-servo-${sid}`);
        if (readout) readout.textContent = pos;

        if (this.activeSlider !== sid) {
          const slider = document.querySelector(`.servo-slider[data-servo="${sid}"]`);
          if (slider) slider.value = pos;
        }
      }
    }

    // Line Sensors
    if (data.line_sensors && data.line_sensors.length === 4) {
      data.line_sensors.forEach((state, i) => {
        if (this.irSensors[i]) {
          if (state === 1) {
            this.irSensors[i].classList.add('active');
          } else {
            this.irSensors[i].classList.remove('active');
          }
        }
      });
    }

    // IMU
    if (data.pitch !== undefined) this.imuPitch.textContent = `${data.pitch.toFixed(1)}°`;
    if (data.roll !== undefined) this.imuRoll.textContent = `${data.roll.toFixed(1)}°`;
    if (data.is_level !== undefined) {
      this.imuStatus.textContent = data.is_level ? 'LEVEL' : 'TILTED';
      this.imuStatus.className = data.is_level ? 'proximity-pill safe' : 'proximity-pill warn';
    }

    // Battery
    if (data.battery_mv) {
      const v = (data.battery_mv / 1000.0).toFixed(1);
      this.batVoltage.textContent = `${v}V (${data.battery_pct}%)`;
    }

    // Mode Sync
    if (data.mode) {
      const modeBtns = document.querySelectorAll('.mode-btn');
      modeBtns.forEach(btn => {
        if (btn.dataset.mode === data.mode) {
          btn.classList.add('active');
        } else {
          btn.classList.remove('active');
        }
      });
      this.curModeLabel.textContent = data.mode;
    }
  }
}

// Initialize on DOM load
document.addEventListener('DOMContentLoaded', () => {
  window.tankbotClient = new TankbotClient();
});
