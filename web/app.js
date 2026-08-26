"use strict";

const LIMITS = {
  joints: [
    { name: "joint1", min: -1.396263, max: 3.490659 },
    { name: "joint2", min: -1.745329, max: 1.745329 },
    { name: "joint3", min: -1.570796, max: 1.570796 },
    { name: "joint4", min: 0.0, max: 2.443461 },
    { name: "joint5", min: -1.570796, max: 1.570796 },
    { name: "joint6", min: -0.785398, max: 0.785398 },
    { name: "joint7", min: -1.570796, max: 1.570796 },
  ],
};

const CHASSIS = {
  rateHz: 50,
  deadzone: 0.08,
  maxVx: 0.5,
  maxVy: 0.5,
  maxWz: 1.0,
};

const state = {
  ws: null,
  wsUrl: "",
  seq: 0,
  activeArm: "left",
  activeCartArm: "left",
  manualClose: false,
  resetChassis: null,
  targets: {
    left: LIMITS.joints.map(() => 0),
    right: LIMITS.joints.map(() => 0),
  },
  gripper: {
    left: 0,
    right: 0,
  },
  chassis: {
    moveX: 0,
    moveY: 0,
    turnX: 0,
  },
};

const $ = (id) => document.getElementById(id);

function defaultBridgeUrl() {
  const url = new URL(window.location.href);
  const explicit = url.searchParams.get("bridge");
  if (explicit) return explicit;
  if (window.location.protocol === "file:") return "ws://127.0.0.1:9000";
  const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${wsProtocol}//${window.location.host}/ws/robot`;
}

function nextSeq() {
  state.seq += 1;
  return state.seq;
}

function log(message, level = "info") {
  const row = document.createElement("div");
  row.className = "log-entry";
  row.innerHTML = `<span class="log-time">${new Date().toLocaleTimeString()}</span><span class="log-${level}">${message}</span>`;
  $("logList").prepend(row);
  while ($("logList").children.length > 80) $("logList").lastChild.remove();
}

function setConnection(kind, label) {
  const el = $("connectionState");
  el.classList.remove("connected", "error");
  if (kind) el.classList.add(kind);
  $("connLabel").textContent = label;
  const url = state.wsUrl || defaultBridgeUrl();
  $("bridgeUrl").textContent = url;
  el.title = `点击修改连接\n${url}`;
}

function normalizeWsUrl(value) {
  const text = value.trim();
  if (!text) return defaultBridgeUrl();
  if (text.startsWith("ws://") || text.startsWith("wss://")) return text;
  return `ws://${text}`;
}

function openConnectModal() {
  $("connectUrl").value = state.wsUrl || defaultBridgeUrl();
  $("connectModal").classList.remove("hidden");
  $("connectUrl").focus();
  $("connectUrl").select();
}

function closeConnectModal() {
  $("connectModal").classList.add("hidden");
}

function connect(url = defaultBridgeUrl()) {
  const nextUrl = normalizeWsUrl(url);
  state.wsUrl = nextUrl;
  $("connectUrl").value = nextUrl;
  setConnection("", "连接中");

  if (state.ws) {
    state.manualClose = true;
    state.ws.close();
  }

  const ws = new WebSocket(nextUrl);
  state.ws = ws;
  state.manualClose = false;

  ws.addEventListener("open", () => {
    if (state.ws !== ws) return;
    setConnection("connected", "已连接");
    log(`已连接 ${nextUrl}`, "ok");
  });

  ws.addEventListener("close", () => {
    if (state.ws !== ws || state.manualClose) return;
    setConnection("error", "已断开");
    log("WebSocket 已断开。若连接打开后立即关闭，请检查 robot_bridge 是否已启动。", "warn");
  });

  ws.addEventListener("error", () => {
    if (state.ws !== ws) return;
    setConnection("error", "连接错误");
    log(`连接错误: ${nextUrl}`, "error");
  });

  ws.addEventListener("message", (event) => {
    try {
      handleFrame(JSON.parse(event.data));
    } catch (err) {
      log(`收到非法 JSON: ${err.message}`, "error");
    }
  });
}

function send(frame) {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
    log("WebSocket 未连接，命令未发送", "error");
    openConnectModal();
    return false;
  }
  state.ws.send(JSON.stringify(frame));
  return true;
}

function sendCommand(target, data) {
  return send({ type: "command", seq: nextSeq(), time: Date.now() / 1000, target, data });
}

function sendRequest(action, data = {}) {
  const seq = nextSeq();
  return send({ type: "request", seq, id: `web-${seq}`, time: Date.now() / 1000, action, data });
}

function handleFrame(frame) {
  if (frame.type === "state") {
    applyRobotState(frame.data || {});
    return;
  }
  if (frame.type === "response") {
    log(`${frame.action || "request"}: ${frame.message || "ok"}`, frame.ok ? "ok" : "warn");
    return;
  }
  if (frame.type === "event") {
    log(`event ${frame.event}: ${JSON.stringify(frame.data || {})}`, frame.event?.includes("failed") ? "error" : "ok");
    return;
  }
  if (frame.type === "error") {
    log(`${frame.code || "ERROR"}: ${frame.message || ""}`, "error");
    setConnection("error", "已断开");
  }
}

function applyRobotState(data) {
  $("armModeState").textContent = data.arm_mode || "trajectory";
  const lift = data.lift || {};
  const waist = data.waist || {};
  const chassis = data.chassis || {};
  $("leftGripperState").textContent = state.gripper.left.toFixed(2);
  $("rightGripperState").textContent = state.gripper.right.toFixed(2);
  $("waistPositionState").textContent = Number.isFinite(waist.position) ? `${waist.position.toFixed(3)} rad` : "--";
  $("waistVelocityState").textContent = Number.isFinite(waist.velocity) ? `${waist.velocity.toFixed(3)} rad/s` : "--";
  $("waistTorqueState").textContent = Number.isFinite(waist.torque) ? `${waist.torque.toFixed(1)} N·m` : "--";
  $("liftPositionState").textContent = Number.isFinite(lift.position) ? `${lift.position.toFixed(1)} mm` : "--";
  $("liftVelocityState").textContent = Number.isFinite(lift.velocity) ? `${lift.velocity.toFixed(1)} mm/s` : "--";
  $("liftEnabledState").textContent = typeof lift.enabled === "boolean" ? (lift.enabled ? "已使能" : "未使能") : "--";
  applyChassisState(chassis);
  applyEePoses(data.ee_poses);
}

function formatBatteryTemps(chassis) {
  const values = [chassis.battery_temp1_c, chassis.battery_temp2_c]
    .filter(Number.isFinite)
    .map((value) => `${value.toFixed(1)}℃`);
  return values.length ? values.join(" / ") : "--";
}

function applyChassisState(chassis) {
  const stateNames = {
    init: "初始化",
    disabled: "已失能",
    enabled: "已使能",
    brake: "刹车",
    estop: "急停",
  };
  const name = chassis.state_name || "";
  const temperature = formatBatteryTemps(chassis);
  $("batteryState").textContent = Number.isFinite(chassis.battery_soc) ? `${chassis.battery_soc}%` : "--";
  $("voltageState").textContent = Number.isFinite(chassis.battery_voltage_v)
    ? `${chassis.battery_voltage_v.toFixed(1)}V` : "--";
  $("batteryTempState").textContent = temperature;
  $("chassisStatusState").textContent = stateNames[name] || name || "--";
  $("chassisVelocityState").textContent =
    [chassis.vx, chassis.vy, chassis.wz].every(Number.isFinite)
      ? `vx ${chassis.vx.toFixed(2)} · vy ${chassis.vy.toFixed(2)} · wz ${chassis.wz.toFixed(2)}`
      : "--";
  document.querySelectorAll("[data-chassis-command]").forEach((button) => {
    const activeState = button.dataset.chassisCommand === "disable" ? "disabled" : button.dataset.chassisCommand;
    button.classList.toggle("active", name === activeState);
  });
}

function fmtVec(values, digits = 3) {
  if (!Array.isArray(values)) return "--";
  return values.map((v) => (Number.isFinite(v) ? Number(v).toFixed(digits) : "--")).join(", ");
}

function applyEePoses(ee) {
  const empty = { position: null, orientation: null };
  const left = (ee && ee.left) || empty;
  const right = (ee && ee.right) || empty;
  const lp = left.position, lq = left.orientation;
  const rp = right.position, rq = right.orientation;
  $("leftEePosState").textContent = lp ? fmtVec([lp.x, lp.y, lp.z], 3) : "--";
  $("leftEeQuatState").textContent = lq ? fmtVec([lq.x, lq.y, lq.z, lq.w], 2) : "--";
  $("rightEePosState").textContent = rp ? fmtVec([rp.x, rp.y, rp.z], 3) : "--";
  $("rightEeQuatState").textContent = rq ? fmtVec([rq.x, rq.y, rq.z, rq.w], 2) : "--";
}

function formatRad(value) {
  return `${Number(value).toFixed(3)} rad`;
}

function paintRange(input, color = "var(--accent)") {
  const min = Number(input.min);
  const max = Number(input.max);
  const pct = (((Number(input.value) - min) / (max - min)) * 100).toFixed(1);
  input.style.background = `linear-gradient(to right, ${color} ${pct}%, var(--surface-3) ${pct}%)`;
}

function bindRange(id, outputId, formatter, color) {
  const input = $(id);
  const update = () => {
    $(outputId).textContent = formatter(input.value);
    paintRange(input, color);
  };
  input.addEventListener("input", update);
  update();
}

function renderJoints() {
  const list = $("jointList");
  list.innerHTML = "";
  LIMITS.joints.forEach((joint, index) => {
    const value = state.targets[state.activeArm][index];
    const row = document.createElement("div");
    row.className = "joint-row";
    row.innerHTML = `
      <div class="joint-name">${joint.name}</div>
      <div>
        <div class="joint-meta">
          <span>${joint.min.toFixed(3)}</span>
          <span class="joint-value" id="jointValue${index}">${formatRad(value)}</span>
          <span>${joint.max.toFixed(3)}</span>
        </div>
        <input id="joint${index}" type="range" min="${joint.min}" max="${joint.max}" step="0.001" value="${value}">
      </div>`;
    list.appendChild(row);
    const input = $(`joint${index}`);
    paintRange(input, "var(--blue)");
    input.addEventListener("input", () => {
      const next = Number(input.value);
      state.targets[state.activeArm][index] = next;
      $(`jointValue${index}`).textContent = formatRad(next);
      paintRange(input, "var(--blue)");
    });
  });
}

function setActivePanel(panelId) {
  document.querySelectorAll(".mode-tab").forEach((tab) => tab.classList.toggle("active", tab.dataset.panel === panelId));
  document.querySelectorAll(".control-panel").forEach((panel) => panel.classList.toggle("hidden", panel.id !== panelId));
}

function bindControls() {
  $("connectionState").addEventListener("click", openConnectModal);
  $("closeConnectModal").addEventListener("click", closeConnectModal);
  $("connectModal").addEventListener("click", (event) => {
    if (event.target === $("connectModal")) closeConnectModal();
  });
  $("connectForm").addEventListener("submit", (event) => {
    event.preventDefault();
    connect($("connectUrl").value);
    closeConnectModal();
  });

  $("startupBtn").addEventListener("click", () => sendRequest("startup", {
    arm_mode: $("startupArmMode").value || "trajectory",
    show_terminal: false,
    components: ["arms", "chassis", "lift", "waist", "emergency_stop"],
    ready_timeout_sec: 30,
  }));
  $("stopBtn").addEventListener("click", () => {
    state.resetChassis?.();
    sendRequest("emergency_stop");
  });
  $("resetBtn").addEventListener("click", () => {
    state.resetChassis?.();
    sendRequest("reset");
  });
  document.querySelectorAll("[data-chassis-command]").forEach((button) => {
    button.addEventListener("click", () => sendChassisState(button.dataset.chassisCommand));
  });

  document.querySelectorAll(".mode-tab").forEach((tab) => tab.addEventListener("click", () => setActivePanel(tab.dataset.panel)));
  document.querySelectorAll(".arm-tab[data-arm]").forEach((tab) => {
    tab.addEventListener("click", () => {
      state.activeArm = tab.dataset.arm;
      document.querySelectorAll(".arm-tab[data-arm]").forEach((item) => item.classList.toggle("active", item === tab));
      renderJoints();
    });
  });
  document.querySelectorAll("[data-cart-arm]").forEach((tab) => {
    tab.addEventListener("click", () => {
      state.activeCartArm = tab.dataset.cartArm;
      document.querySelectorAll("[data-cart-arm]").forEach((item) => item.classList.toggle("active", item === tab));
      log(`笛卡尔控制切换到 ${state.activeCartArm === "left" ? "左臂" : "右臂"}`, "info");
    });
  });

  $("cartHomeBtn").addEventListener("click", () => {
    // 只改滑块数据，不发送；左右臂原点不同
    const home = state.activeCartArm === "right"
      ? { x: 0.0, y: -0.15, z: 0.16 }
      : { x: 0.0, y: 0.15, z: 0.16 };
    const defaults = {
      cartPosX: home.x, cartPosY: home.y, cartPosZ: home.z,
      cartOriX: 1.0, cartOriY: 0.0, cartOriZ: 0.0, cartOriW: 0.0,
      cartVelScale: 0.2, cartAccScale: 0.2,
    };
    for (const [id, value] of Object.entries(defaults)) {
      const input = $(id);
      input.value = value;
      input.dispatchEvent(new Event("input", { bubbles: true }));
    }
    log(`笛卡尔滑块已重置为${state.activeCartArm === "left" ? "左" : "右"}臂原点（未发送）`, "warn");
  });

  $("sendCartesianBtn").addEventListener("click", () => {
    const data = {
      arm: state.activeCartArm,
      position: {
        x: Number($("cartPosX").value),
        y: Number($("cartPosY").value),
        z: Number($("cartPosZ").value),
      },
      orientation: {
        x: Number($("cartOriX").value),
        y: Number($("cartOriY").value),
        z: Number($("cartOriZ").value),
        w: Number($("cartOriW").value),
      },
      vel_scale: Number($("cartVelScale").value),
      acc_scale: Number($("cartAccScale").value),
      plan_only: $("cartPlanOnly").checked,
      frame_id: "",
    };
    if (sendCommand("arm_cartesian", data)) {
      log(`已发送笛卡尔目标(${data.arm}) pos=(${data.position.x.toFixed(3)},${data.position.y.toFixed(3)},${data.position.z.toFixed(3)}) plan_only=${data.plan_only}`, "ok");
    }
  });

  $("sendArmsBtn").addEventListener("click", () => {
    const duration = Number($("trajectoryDuration").value || 2);
    if (sendCommand("arm_traj", { left: state.targets.left, right: state.targets.right, duration })) log(`已发送双臂轨迹 duration=${duration}s`, "ok");
  });
  $("zeroArmsBtn").addEventListener("click", () => {
    state.targets.left = LIMITS.joints.map(() => 0);
    state.targets.right = LIMITS.joints.map(() => 0);
    renderJoints();
    log("双臂目标已归零，尚未发送", "warn");
  });
  $("sendLeftGripperBtn").addEventListener("click", () => {
    const value = Number($("leftGripper").value);
    state.gripper.left = value;
    $("leftGripperState").textContent = value.toFixed(2);
    if (sendCommand("gripper", { left: value })) log(`已发送左夹爪 ${value.toFixed(2)}`, "ok");
  });
  $("sendRightGripperBtn").addEventListener("click", () => {
    const value = Number($("rightGripper").value);
    state.gripper.right = value;
    $("rightGripperState").textContent = value.toFixed(2);
    if (sendCommand("gripper", { right: value })) log(`已发送右夹爪 ${value.toFixed(2)}`, "ok");
  });
  $("sendWaistBtn").addEventListener("click", () => {
    const data = {
      position: Number($("waistPosition").value),
      velocity: Number($("waistVelocity").value),
      torque: Number($("waistTorque").value),
    };
    if (sendCommand("waist", data)) log(`已发送腰部 ${JSON.stringify(data)}`, "ok");
  });
  $("sendLiftVelocityBtn").addEventListener("click", () => {
    const velocity = Number($("liftVelocity").value);
    if (sendCommand("lift", { velocity })) log(`已设置升降速度 ${velocity} mm/s`, "ok");
  });
  $("sendLiftPositionBtn").addEventListener("click", () => {
    const position = Number($("liftPosition").value);
    if (sendCommand("lift", { position })) log(`已发送升降目标 ${position} mm`, "ok");
  });
}

function bindSliders() {
  bindRange("baseSpeed", "baseSpeedValue", (v) => `${Math.round(Number(v) * 100)}%`);
  bindRange("leftGripper", "leftGripperValue", (v) => Number(v).toFixed(2));
  bindRange("rightGripper", "rightGripperValue", (v) => Number(v).toFixed(2));
  bindRange("waistPosition", "waistPositionValue", formatRad);
  bindRange("waistVelocity", "waistVelocityValue", (v) => `${Number(v).toFixed(3)} rad/s`);
  bindRange("waistTorque", "waistTorqueValue", (v) => `${Number(v).toFixed(0)} N·m`);
  bindRange("liftPosition", "liftPositionValue", (v) => `${Number(v).toFixed(0)} mm`);
  bindRange("liftVelocity", "liftVelocityValue", (v) => `${Number(v).toFixed(0)} mm/s`);
  bindRange("cartPosX", "cartPosXValue", (v) => Number(v).toFixed(3));
  bindRange("cartPosY", "cartPosYValue", (v) => Number(v).toFixed(3));
  bindRange("cartPosZ", "cartPosZValue", (v) => Number(v).toFixed(3));
  bindRange("cartOriX", "cartOriXValue", (v) => Number(v).toFixed(3));
  bindRange("cartOriY", "cartOriYValue", (v) => Number(v).toFixed(3));
  bindRange("cartOriZ", "cartOriZValue", (v) => Number(v).toFixed(3));
  bindRange("cartOriW", "cartOriWValue", (v) => Number(v).toFixed(3));
  bindRange("cartVelScale", "cartVelScaleValue", (v) => Number(v).toFixed(2));
  bindRange("cartAccScale", "cartAccScaleValue", (v) => Number(v).toFixed(2));
}

function applyDeadzone(value) {
  const magnitude = Math.abs(value);
  if (magnitude <= CHASSIS.deadzone) return 0;
  return Math.sign(value) * (magnitude - CHASSIS.deadzone) / (1 - CHASSIS.deadzone);
}

function bindJoystick(areaId, knobId, onMove) {
  const area = $(areaId);
  const knob = $(knobId);
  let active = false;

  function move(clientX, clientY) {
    const rect = area.getBoundingClientRect();
    // 旋钮中心可推到接近外圈，最大化有效行程
    const radius = Math.max(1, Math.min(rect.width, rect.height) / 2 - 2);
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > radius) {
      dx = (dx / dist) * radius;
      dy = (dy / dist) * radius;
    }
    knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    onMove(dx / radius, -dy / radius);
  }

  function reset() {
    knob.style.transform = "translate(-50%, -50%)";
    onMove(0, 0);
    active = false;
  }

  area.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    active = true;
    area.setPointerCapture(event.pointerId);
    move(event.clientX, event.clientY);
  });
  area.addEventListener("pointermove", (event) => {
    if (active) move(event.clientX, event.clientY);
  });
  area.addEventListener("pointerup", reset);
  area.addEventListener("pointercancel", reset);
  return reset;
}

function bindChassisControl() {
  const resetMove = bindJoystick("moveJoy", "moveJoyKnob", (x, y) => {
    state.chassis.moveX = x;
    state.chassis.moveY = y;
  });

  const stop = () => {
    resetMove();
    state.chassis.turnX = 0;
  };

  const brakeButton = document.querySelector('[data-base="brake"]');
  brakeButton.addEventListener("click", () => sendChassisState("brake"));

  const bindHold = (name, onDown, onUp) => {
    const button = document.querySelector(`[data-base="${name}"]`);
    const down = (event) => {
      event.preventDefault();
      onDown();
    };
    button.addEventListener("pointerdown", down);
    button.addEventListener("pointerup", onUp);
    button.addEventListener("pointercancel", onUp);
    button.addEventListener("pointerleave", onUp);
    button.addEventListener("keydown", (event) => {
      if (event.key === " " || event.key === "Enter") down(event);
    });
    button.addEventListener("keyup", onUp);
  };

  bindHold("rotate-left", () => { state.chassis.turnX = -1; }, () => {
    if (state.chassis.turnX === -1) state.chassis.turnX = 0;
  });
  bindHold("rotate-right", () => { state.chassis.turnX = 1; }, () => {
    if (state.chassis.turnX === 1) state.chassis.turnX = 0;
  });
  state.resetChassis = stop;
  window.addEventListener("blur", stop);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stop();
  });

  setInterval(() => {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
    const scale = Number($("baseSpeed").value);
    const data = {
      vx: applyDeadzone(state.chassis.moveY) * CHASSIS.maxVx * scale,
      vy: -applyDeadzone(state.chassis.moveX) * CHASSIS.maxVy * scale,
      wz: -applyDeadzone(state.chassis.turnX) * CHASSIS.maxWz * scale,
    };
    sendCommand("chassis", data);
  }, 1000 / CHASSIS.rateHz);
}

function sendChassisState(command) {
  if (command === "brake" || command === "disable") state.resetChassis?.();
  if (sendCommand("chassis_state", { command })) {
    const labels = { brake: "刹车", enable: "使能", disable: "失能" };
    log(`已发送底盘${labels[command] || command}指令`, "ok");
  }
}

function init() {
  state.wsUrl = defaultBridgeUrl();
  bindControls();
  bindSliders();
  bindChassisControl();
  renderJoints();
  log("页面初始化完成，点击顶部连接状态可修改地址", "ok");
  connect(state.wsUrl);
}

document.addEventListener("DOMContentLoaded", init);
