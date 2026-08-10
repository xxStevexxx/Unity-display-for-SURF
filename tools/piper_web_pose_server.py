#!/usr/bin/env python3
"""Web interface for controlling and previewing the PiPER arm via ROS topics.

Serves a simple HTML form at the root URL and exposes a POST endpoint ``/move``
that runs ``piper_moveit_pose_planner.py`` with the user-supplied target pose.
The live status panel reads ROS topic data directly via ``ros2 topic`` or
``rostopic`` and renders an abstract canvas linkage preview.
Uses only the Python standard library so it works inside the ROS2 Docker image
without extra pip packages.
"""

import argparse
import html
import json
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8765
STATUS_POLL_INTERVAL = float(os.environ.get("WEB_POSE_STATUS_POLL_INTERVAL", "0.5"))
STATUS_CACHE: dict[str, object] = {
    "ok": False,
    "updated_at": 0.0,
    "error": "status sampler not started",
}
STATUS_CACHE_LOCK = threading.Lock()
ROS_MONITOR = None
JOG_LOCK = threading.Lock()

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PiPER ROS 坐标控制</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #eef2f6;
      --panel: #ffffff;
      --panel-soft: #f8fafc;
      --border: #d8dee8;
      --text: #172033;
      --muted: #667085;
      --good: #15803d;
      --warn: #b45309;
      --bad: #b91c1c;
      --blue: #1d4ed8;
      --blue-soft: #dbeafe;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    .shell { max-width: 1280px; margin: 0 auto; padding: 18px; }
    header { display: flex; justify-content: space-between; gap: 16px; align-items: flex-end; margin-bottom: 18px; }
    h1 { margin: 0; font-size: 24px; line-height: 1.2; }
    .subtitle { margin-top: 6px; color: var(--muted); font-size: 14px; }
    .toolbar { display: flex; gap: 8px; align-items: center; }
    button {
      border: 1px solid #1d4ed8;
      background: var(--blue);
      color: white;
      border-radius: 6px;
      padding: 9px 13px;
      font-size: 14px;
      cursor: pointer;
    }
    button.secondary { background: white; color: var(--blue); }
    button:disabled { opacity: 0.55; cursor: wait; }
    .grid { display: grid; grid-template-columns: minmax(340px, 0.85fr) minmax(520px, 1.15fr); gap: 16px; align-items: start; }
    .panel {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 16px;
      box-shadow: 0 1px 2px rgba(16, 24, 40, 0.06);
    }
    .panel h2 { margin: 0 0 12px; font-size: 16px; }
    .status-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    .status-card { border: 1px solid var(--border); border-radius: 8px; padding: 12px; background: var(--panel-soft); min-height: 92px; }
    .status-head { display: flex; justify-content: space-between; gap: 8px; align-items: center; margin-bottom: 8px; }
    .status-title { font-weight: 650; font-size: 14px; }
    .pill { display: inline-flex; align-items: center; min-height: 24px; border-radius: 999px; padding: 3px 8px; font-size: 12px; font-weight: 650; }
    .pill.ok { color: var(--good); background: #dcfce7; }
    .pill.warn { color: var(--warn); background: #fef3c7; }
    .pill.bad { color: var(--bad); background: #fee2e2; }
    .pill.neutral { color: #475467; background: #e5e7eb; }
    .kv { display: grid; grid-template-columns: 88px 1fr; gap: 5px 8px; font-size: 13px; }
    .kv .key { color: var(--muted); }
    .kv .value { font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .pose-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 10px; }
    .preview-wrap { margin-top: 12px; border: 1px solid var(--border); border-radius: 8px; background: #f8fafc; padding: 10px; }
    .preview-head { display: flex; justify-content: space-between; gap: 10px; align-items: center; margin-bottom: 8px; color: var(--muted); font-size: 12px; }
    #armPreview { width: 100%; height: min(560px, 58vh); min-height: 360px; display: block; background: #ffffff; border: 1px solid var(--border); border-radius: 6px; overflow: hidden; touch-action: none; cursor: grab; position: relative; }
    #armPreview.dragging { cursor: grabbing; }
    .metric { border: 1px solid var(--border); border-radius: 6px; padding: 9px; background: white; }
    .metric .label { color: var(--muted); font-size: 12px; }
    .metric .value { margin-top: 3px; font-size: 18px; font-weight: 700; font-variant-numeric: tabular-nums; }
    .field { margin-bottom: 12px; }
    label { display: block; margin-bottom: 5px; font-weight: 650; font-size: 13px; }
    input[type="number"], input[type="text"] {
      width: 100%; padding: 9px 10px; border: 1px solid #cbd5e1; border-radius: 6px; font-size: 14px; background: white;
    }
    .row { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .target-picker { display: grid; grid-template-columns: minmax(240px, 1fr) 76px; gap: 14px; align-items: stretch; margin-bottom: 14px; }
    .xy-picker { border: 1px solid var(--border); border-radius: 8px; background: #ffffff; padding: 10px; }
    .xy-head { display: flex; justify-content: space-between; gap: 10px; align-items: center; margin-bottom: 8px; color: var(--muted); font-size: 12px; }
    #xyPad { width: 100%; aspect-ratio: 1 / 1; display: block; border: 1px solid #cbd5e1; border-radius: 6px; background: #f8fafc; touch-action: none; cursor: crosshair; }
    .z-picker { border: 1px solid var(--border); border-radius: 8px; background: var(--panel-soft); padding: 10px; display: grid; grid-template-rows: auto 1fr auto; gap: 8px; align-items: center; justify-items: center; min-height: 280px; }
    .z-picker label { margin: 0; }
    .z-picker input[type="range"] { writing-mode: vertical-rl; direction: rtl; width: 26px; height: 210px; }
    .picker-actions { display: flex; flex-wrap: wrap; gap: 8px; margin: 4px 0 14px; }
    .picker-note { color: var(--muted); font-size: 12px; line-height: 1.4; }
    .checkline { display: flex; align-items: center; gap: 8px; margin: 8px 0 14px; color: #344054; font-size: 14px; }
    .checkline input { width: 16px; height: 16px; }
    .orientation-row.disabled { opacity: 0.48; }
    .orientation-row.disabled input { background: #f1f5f9; }
    .jog-panel { border: 1px solid var(--border); border-radius: 8px; background: var(--panel-soft); padding: 10px; margin: 4px 0 14px; }
    .jog-head { display: flex; justify-content: space-between; gap: 10px; align-items: center; margin-bottom: 8px; }
    .jog-title { font-weight: 650; font-size: 14px; }
    .jog-status { color: var(--muted); font-size: 12px; font-variant-numeric: tabular-nums; }
    .jog-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .jog-card { border: 1px solid var(--border); border-radius: 6px; background: white; padding: 8px; }
    .jog-card .axis { display: flex; justify-content: space-between; color: var(--muted); font-size: 12px; margin-bottom: 7px; }
    .jog-buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
    .jog-button { padding: 8px 0; font-size: 15px; font-weight: 700; }
    .jog-settings { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; margin-top: 8px; }
    .hint { color: var(--muted); font-size: 13px; line-height: 1.45; margin: 8px 0 0; }
    .joint-table { width: 100%; border-collapse: collapse; font-size: 13px; margin-top: 10px; }
    .joint-table th, .joint-table td { border-bottom: 1px solid var(--border); padding: 7px 6px; text-align: right; font-variant-numeric: tabular-nums; }
    .joint-table th:first-child, .joint-table td:first-child { text-align: left; color: var(--muted); }
    .topics { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 10px; }
    .topic { border: 1px solid var(--border); border-radius: 999px; padding: 4px 8px; font-size: 12px; color: var(--muted); background: white; }
    .topic.ok { color: var(--good); border-color: #86efac; background: #f0fdf4; }
    .result { margin-top: 12px; padding: 12px; background: #0f172a; color: #e2e8f0; border-radius: 6px; white-space: pre-wrap; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 12px; max-height: 280px; overflow: auto; }
    .result.success { background: #ecfdf3; color: var(--good); border: 1px solid #bbf7d0; }
    .result.error { background: #fef2f2; color: var(--bad); border: 1px solid #fecaca; }
    @media (max-width: 900px) {
      .shell { padding: 14px; }
      header { display: block; }
      .toolbar { margin-top: 12px; }
      .grid, .status-grid, .row, .target-picker { grid-template-columns: 1fr; }
      #armPreview { height: 420px; min-height: 320px; }
      .z-picker { min-height: auto; justify-items: stretch; }
      .z-picker input[type="range"] { writing-mode: horizontal-tb; width: 100%; height: auto; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header>
      <div>
        <h1>PiPER ROS 坐标控制</h1>
        <div class="subtitle">MoveIt 目标位姿执行 · 连杆 FK · ROS 反馈监听 · 真实机器状态检查</div>
      </div>
      <div class="toolbar">
        <span id="lastUpdate" class="pill neutral">等待刷新</span>
        <span id="autoRefresh" class="pill ok">自动刷新 0.5s</span>
        <button class="secondary" id="refreshBtn" type="button">刷新状态</button>
      </div>
    </header>

    <div class="grid">
      <section class="panel">
        <h2>目标位姿</h2>
        <form id="poseForm" method="post" action="/move">
          <div class="target-picker">
            <div class="xy-picker">
              <div class="xy-head">
                <span>XY 选点 · base_link 平面</span>
                <span id="xyReadout">X 0.300 · Y 0.000</span>
              </div>
              <canvas id="xyPad" width="420" height="420"></canvas>
            </div>
            <div class="z-picker">
              <label for="zRange">Z</label>
              <input type="range" id="zRange" min="0.02" max="0.60" step="0.001" value="0.4">
              <span id="zReadout">0.400 m</span>
            </div>
          </div>
          <div class="picker-actions">
            <button class="secondary" id="useCurrentBtn" type="button">使用当前位姿</button>
            <button class="secondary" id="centerCurrentBtn" type="button">当前点居中</button>
            <button class="secondary" id="centerOriginBtn" type="button">原点居中</button>
            <span class="picker-note">拖动画布设置 X/Y，滑条设置 Z；下方数值框会同步，点击“规划并执行”后由 MoveIt 解算和执行。</span>
          </div>
          <div class="row">
            <div class="field"><label for="x">X (m)</label><input type="number" id="x" name="x" step="0.001" value="0.3" required></div>
            <div class="field"><label for="y">Y (m)</label><input type="number" id="y" name="y" step="0.001" value="0.0" required></div>
            <div class="field"><label for="z">Z (m)</label><input type="number" id="z" name="z" step="0.001" value="0.4" required></div>
          </div>
          <div class="row orientation-row" id="orientationRow">
            <div class="field"><label for="roll">Roll (deg)</label><input type="number" id="roll" name="roll" step="0.1" value="0.0"></div>
            <div class="field"><label for="pitch">Pitch (deg)</label><input type="number" id="pitch" name="pitch" step="0.1" value="0.0"></div>
            <div class="field"><label for="yaw">Yaw (deg)</label><input type="number" id="yaw" name="yaw" step="0.1" value="0.0"></div>
          </div>
          <div class="row">
            <div class="field"><label for="planner">规划器</label><input type="text" id="planner" name="planner" value="" placeholder="默认"></div>
            <div class="field"><label for="planning_time">规划时间 (s)</label><input type="number" id="planning_time" name="planning_time" step="0.1" min="0.1" value="5.0"></div>
            <div class="field"><label for="velocity_scaling">速度缩放</label><input type="number" id="velocity_scaling" name="velocity_scaling" step="0.05" min="0.01" max="1.0" value="0.3"></div>
          </div>
          <label class="checkline"><input type="checkbox" id="position_only" name="position_only" value="1" checked> 末端姿态不限模式</label>
          <div class="jog-panel">
            <div class="jog-head">
              <span class="jog-title">实时控制模式</span>
              <span id="jogStatus" class="jog-status">待命</span>
            </div>
            <div class="jog-grid">
              <div class="jog-card"><div class="axis"><span>X</span><span>A / D</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="x" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="x" data-jog-dir="1">+</button></div></div>
              <div class="jog-card"><div class="axis"><span>Y</span><span>S / W</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="y" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="y" data-jog-dir="1">+</button></div></div>
              <div class="jog-card"><div class="axis"><span>Z</span><span>Q / E</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="z" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="z" data-jog-dir="1">+</button></div></div>
              <div class="jog-card"><div class="axis"><span>Roll</span><span>I / K</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="roll" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="roll" data-jog-dir="1">+</button></div></div>
              <div class="jog-card"><div class="axis"><span>Pitch</span><span>J / L</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="pitch" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="pitch" data-jog-dir="1">+</button></div></div>
              <div class="jog-card"><div class="axis"><span>Yaw</span><span>U / O</span></div><div class="jog-buttons"><button class="secondary jog-button" type="button" data-jog-axis="yaw" data-jog-dir="-1">-</button><button class="secondary jog-button" type="button" data-jog-axis="yaw" data-jog-dir="1">+</button></div></div>
            </div>
            <div class="jog-settings">
              <div class="field"><label for="jog_linear_step">线性步长 (m)</label><input type="number" id="jog_linear_step" step="0.001" min="0.001" max="0.05" value="0.01"></div>
              <div class="field"><label for="jog_angular_step">角度步长 (deg)</label><input type="number" id="jog_angular_step" step="0.1" min="0.1" max="10" value="2.0"></div>
              <div class="field"><label for="jog_planning_time">Jog 规划时间 (s)</label><input type="number" id="jog_planning_time" step="0.1" min="0.2" max="5" value="1.0"></div>
            </div>
          </div>
          <button id="submitBtn" type="submit">规划并执行</button>
        </form>
        <p class="hint">执行前建议确认：/arm_status 在线、/joint_states_feedback 或真实 /joint_states 有数据、/link6_pose 正在更新。</p>
        <div id="output" class="result" style="display:none;"></div>
      </section>

      <section class="panel">
        <h2>实时状态</h2>
        <div class="status-grid">
          <div class="status-card">
            <div class="status-head"><span class="status-title">ROS 图</span><span id="rosPill" class="pill neutral">未知</span></div>
            <div class="kv">
              <span class="key">ros2</span><span id="rosCli" class="value">--</span>
              <span class="key">Topic 数</span><span id="topicCount" class="value">--</span>
            </div>
          </div>
          <div class="status-card">
            <div class="status-head"><span class="status-title">真实机器/仿真反馈</span><span id="feedbackPill" class="pill neutral">未知</span></div>
            <div class="kv">
              <span class="key">来源</span><span id="feedbackSource" class="value">--</span>
              <span class="key">年龄</span><span id="feedbackAge" class="value">--</span>
              <span class="key">采样</span><span id="sampleAge" class="value">--</span>
            </div>
          </div>
          <div class="status-card">
            <div class="status-head"><span class="status-title">机械臂状态</span><span id="armPill" class="pill neutral">未知</span></div>
            <div class="kv">
              <span class="key">arm_status</span><span id="armStatus" class="value">--</span>
              <span class="key">motion</span><span id="motionStatus" class="value">--</span>
              <span class="key">err_code</span><span id="errCode" class="value">--</span>
            </div>
          </div>
          <div class="status-card">
            <div class="status-head"><span class="status-title">连杆 FK</span><span id="fkPill" class="pill neutral">未知</span></div>
            <div class="kv">
              <span class="key">frame</span><span id="fkFrame" class="value">--</span>
              <span class="key">年龄</span><span id="fkAge" class="value">--</span>
            </div>
          </div>
        </div>

        <div class="pose-grid">
          <div class="metric"><div class="label">X</div><div class="value" id="px">--</div></div>
          <div class="metric"><div class="label">Y</div><div class="value" id="py">--</div></div>
          <div class="metric"><div class="label">Z</div><div class="value" id="pz">--</div></div>
          <div class="metric"><div class="label">Roll</div><div class="value" id="pr">--</div></div>
          <div class="metric"><div class="label">Pitch</div><div class="value" id="pp">--</div></div>
          <div class="metric"><div class="label">Yaw</div><div class="value" id="pw">--</div></div>
        </div>

        <div class="preview-wrap">
          <div class="preview-head">
            <span id="previewSource">等待关节 topic</span>
            <span id="previewMeta">--</span>
          </div>
          <div id="armPreview"></div>
          <p class="hint">输入和 /link6_pose 数值均为 ROS base_link 坐标；WebGL 只在显示层把 ROS XYZ 转为画面坐标，Unity world XYZ 不参与 MoveIt 目标解算。</p>
        </div>

        <table class="joint-table">
          <thead><tr><th>关节</th><th>rad</th><th>deg</th></tr></thead>
          <tbody id="jointRows"></tbody>
        </table>

        <div class="topics" id="topics"></div>
      </section>
    </div>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/examples/js/loaders/STLLoader.js"></script>
  <script>
    const form = document.getElementById('poseForm');
    const output = document.getElementById('output');
    const submitBtn = document.getElementById('submitBtn');
    const refreshBtn = document.getElementById('refreshBtn');
    const useCurrentBtn = document.getElementById('useCurrentBtn');
    const centerCurrentBtn = document.getElementById('centerCurrentBtn');
    const xyPad = document.getElementById('xyPad');
    const zRange = document.getElementById('zRange');
    const xInput = document.getElementById('x');
    const yInput = document.getElementById('y');
    const zInput = document.getElementById('z');
    const rollInput = document.getElementById('roll');
    const pitchInput = document.getElementById('pitch');
    const yawInput = document.getElementById('yaw');
    const plannerInput = document.getElementById('planner');
    const velocityScalingInput = document.getElementById('velocity_scaling');
    const positionOnlyInput = document.getElementById('position_only');
    const orientationRow = document.getElementById('orientationRow');
    const centerOriginBtn = document.getElementById('centerOriginBtn');
    const jogStatus = document.getElementById('jogStatus');
    const jogLinearStepInput = document.getElementById('jog_linear_step');
    const jogAngularStepInput = document.getElementById('jog_angular_step');
    const jogPlanningTimeInput = document.getElementById('jog_planning_time');
    const requiredTopics = ['/link6_pose', '/joint_states_feedback', '/joint_states', '/arm_status', '/joint_ctrl_cmd', '/enable_cmd'];
    const fmt = (value, digits = 4, suffix = '') => Number.isFinite(value) ? value.toFixed(digits) + suffix : '--';
    const deg = value => Number.isFinite(value) ? (value * 180 / Math.PI).toFixed(1) : '--';
    const setText = (id, value) => { document.getElementById(id).textContent = value; };
    const setPill = (id, state, text) => {
      const el = document.getElementById(id);
      el.className = 'pill ' + state;
      el.textContent = text;
    };
    const xyState = {
      centerX: Number(localStorage.getItem('piper_xy_center_x_m')) || 0.0,
      centerY: Number(localStorage.getItem('piper_xy_center_y_m')) || 0.0,
      range: Number(localStorage.getItem('piper_xy_range_m')) || 1.0,
      dragging: false,
    };
    const jogState = {
      activeKeys: new Map(),
      activePointer: null,
      timer: null,
      busy: false,
      pendingAxis: null,
      pendingDir: 0,
      lastSentAt: 0,
    };

    function setXYCenter(x, y) {
      xyState.centerX = Number.isFinite(x) ? x : 0.0;
      xyState.centerY = Number.isFinite(y) ? y : 0.0;
      localStorage.setItem('piper_xy_center_x_m', String(xyState.centerX));
      localStorage.setItem('piper_xy_center_y_m', String(xyState.centerY));
      drawXYPad();
    }

    function updateOrientationMode() {
      const positionOnly = positionOnlyInput.checked;
      orientationRow.classList.toggle('disabled', positionOnly);
      for (const input of [rollInput, pitchInput, yawInput]) input.disabled = positionOnly;
    }

    function targetFromInputs() {
      return {
        x: Number(xInput.value),
        y: Number(yInput.value),
        z: Number(zInput.value),
        roll: Number(rollInput.value),
        pitch: Number(pitchInput.value),
        yaw: Number(yawInput.value),
      };
    }

    function setJogStatus(text, state = 'neutral') {
      jogStatus.textContent = text;
      jogStatus.style.color = state === 'bad' ? 'var(--bad)' : (state === 'ok' ? 'var(--good)' : 'var(--muted)');
    }

    function setPoseInputs(pose, options = {}) {
      if (Number.isFinite(pose.x)) xInput.value = pose.x.toFixed(3);
      if (Number.isFinite(pose.y)) yInput.value = pose.y.toFixed(3);
      if (Number.isFinite(pose.z)) {
        zInput.value = pose.z.toFixed(3);
        zRange.value = Math.max(Number(zRange.min), Math.min(Number(zRange.max), pose.z)).toFixed(3);
      }
      if (Number.isFinite(pose.roll)) rollInput.value = pose.roll.toFixed(1);
      if (Number.isFinite(pose.pitch)) pitchInput.value = pose.pitch.toFixed(1);
      if (Number.isFinite(pose.yaw)) yawInput.value = pose.yaw.toFixed(1);
      if (options.center) {
        setXYCenter(Number(xInput.value), Number(yInput.value));
        return;
      }
      drawXYPad();
    }

    function applyJogToInputs(axis, direction) {
      const linearStep = Math.max(0.001, Math.min(0.05, Number(jogLinearStepInput.value) || 0.01));
      const angularStep = Math.max(0.1, Math.min(10, Number(jogAngularStepInput.value) || 2.0));
      const inputByAxis = { x: xInput, y: yInput, z: zInput, roll: rollInput, pitch: pitchInput, yaw: yawInput };
      const input = inputByAxis[axis];
      if (!input) return null;

      if (['roll', 'pitch', 'yaw'].includes(axis) && positionOnlyInput.checked) {
        positionOnlyInput.checked = false;
        updateOrientationMode();
      }

      const step = ['x', 'y', 'z'].includes(axis) ? linearStep : angularStep;
      const current = Number(input.value) || 0;
      input.value = (current + direction * step).toFixed(['x', 'y', 'z'].includes(axis) ? 3 : 1);
      if (axis === 'z') {
        zRange.value = Math.max(Number(zRange.min), Math.min(Number(zRange.max), Number(zInput.value))).toFixed(3);
      }
      drawXYPad();
      return targetFromInputs();
    }

    async function sendJog(axis, direction) {
      if (jogState.busy) {
        jogState.pendingAxis = axis;
        jogState.pendingDir = direction;
        return;
      }

      const target = applyJogToInputs(axis, direction);
      if (!target) return;

      jogState.busy = true;
      jogState.lastSentAt = performance.now();
      setJogStatus(`${axis} ${direction > 0 ? '+' : '-'} 发送中`);
      try {
        const formData = new FormData();
        for (const [key, value] of Object.entries(target)) formData.set(key, String(value));
        formData.set('planner', plannerInput.value || '');
        formData.set('planning_time', String(Math.max(0.2, Math.min(5, Number(jogPlanningTimeInput.value) || 1.0))));
        formData.set('velocity_scaling', velocityScalingInput.value || '0.3');
        if (positionOnlyInput.checked) formData.set('position_only', '1');

        const response = await fetch('/api/jog', { method: 'POST', body: formData });
        const payload = await response.json();
        if (!response.ok || !payload.ok) throw new Error(payload.error || payload.message || 'jog failed');
        setJogStatus(`完成 ${Math.round(performance.now() - jogState.lastSentAt)} ms`, 'ok');
      } catch (err) {
        setJogStatus(err.message, 'bad');
      } finally {
        jogState.busy = false;
        pollStatus();
        if (jogState.pendingAxis) {
          const nextAxis = jogState.pendingAxis;
          const nextDir = jogState.pendingDir;
          jogState.pendingAxis = null;
          jogState.pendingDir = 0;
          sendJog(nextAxis, nextDir);
        }
      }
    }

    function startJog(axis, direction, pointerId = null) {
      jogState.activePointer = pointerId;
      sendJog(axis, direction);
      if (jogState.timer != null) clearInterval(jogState.timer);
      jogState.timer = setInterval(() => sendJog(axis, direction), 260);
    }

    function stopJog() {
      if (jogState.timer != null) clearInterval(jogState.timer);
      jogState.timer = null;
      jogState.activePointer = null;
      if (!jogState.busy) setJogStatus('待命');
    }

    function setupPickerCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const size = Math.max(260, Math.round(rect.width || 360));
      if (canvas.width !== Math.round(size * dpr) || canvas.height !== Math.round(size * dpr)) {
        canvas.width = Math.round(size * dpr);
        canvas.height = Math.round(size * dpr);
      }
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { ctx, size };
    }

    function worldToPad(x, y, size) {
      const half = xyState.range * 0.5;
      return {
        px: ((x - (xyState.centerX - half)) / xyState.range) * size,
        py: (1 - ((y - (xyState.centerY - half)) / xyState.range)) * size,
      };
    }

    function padToWorld(px, py, size) {
      const half = xyState.range * 0.5;
      return {
        x: xyState.centerX - half + (px / size) * xyState.range,
        y: xyState.centerY - half + (1 - py / size) * xyState.range,
      };
    }

    function drawXYPad() {
      const { ctx, size } = setupPickerCanvas(xyPad);
      const target = targetFromInputs();
      ctx.clearRect(0, 0, size, size);
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, size, size);

      ctx.strokeStyle = '#e5e7eb';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 8; i++) {
        const p = i * size / 8;
        ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, size); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(size, p); ctx.stroke();
      }

      const origin = worldToPad(0, 0, size);
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 2;
      if (origin.px >= 0 && origin.px <= size) {
        ctx.beginPath(); ctx.moveTo(origin.px, 0); ctx.lineTo(origin.px, size); ctx.stroke();
      }
      if (origin.py >= 0 && origin.py <= size) {
        ctx.beginPath(); ctx.moveTo(0, origin.py); ctx.lineTo(size, origin.py); ctx.stroke();
      }

      const marker = worldToPad(target.x, target.y, size);
      const clampedX = Math.max(0, Math.min(size, marker.px));
      const clampedY = Math.max(0, Math.min(size, marker.py));
      ctx.fillStyle = '#1d4ed8';
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(clampedX, clampedY, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.strokeStyle = '#15803d';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(clampedX - 16, clampedY);
      ctx.lineTo(clampedX + 16, clampedY);
      ctx.moveTo(clampedX, clampedY - 16);
      ctx.lineTo(clampedX, clampedY + 16);
      ctx.stroke();

      ctx.fillStyle = '#475467';
      ctx.font = '12px system-ui, sans-serif';
      const half = xyState.range * 0.5;
      ctx.fillText(`X ${fmt(xyState.centerX - half, 2)} ... ${fmt(xyState.centerX + half, 2)} m`, 10, 20);
      ctx.fillText(`Y ${fmt(xyState.centerY - half, 2)} ... ${fmt(xyState.centerY + half, 2)} m`, 10, 38);
      ctx.fillStyle = '#1d4ed8';
      ctx.fillText('+X', size - 28, Math.max(16, origin.py - 8));
      ctx.fillText('+Y', Math.min(size - 24, origin.px + 8), 18);
      setText('xyReadout', `X ${fmt(target.x, 3)} · Y ${fmt(target.y, 3)}`);
      setText('zReadout', `${fmt(Number(zInput.value), 3)} m`);
    }

    function setXYFromEvent(evt) {
      const rect = xyPad.getBoundingClientRect();
      const size = Math.max(1, rect.width);
      const px = Math.max(0, Math.min(size, evt.clientX - rect.left));
      const py = Math.max(0, Math.min(size, evt.clientY - rect.top));
      const world = padToWorld(px, py, size);
      xInput.value = world.x.toFixed(3);
      yInput.value = world.y.toFixed(3);
      drawXYPad();
    }

    async function loadCurrentPose(options = {}) {
      const response = await fetch('/current_pose', { cache: 'no-store' });
      const pose = await response.json();
      if (!response.ok) throw new Error(pose.error || '当前位姿不可用');
      setPoseInputs(pose, options);
      return pose;
    }

    form.addEventListener('submit', async (e) => {
      e.preventDefault();
      output.style.display = 'block';
      output.textContent = '运行中，请稍候...';
      output.className = 'result';
      submitBtn.disabled = true;
      const formData = new FormData(form);
      try {
        const response = await fetch('/move', { method: 'POST', body: formData });
        const text = await response.text();
        output.innerHTML = text;
        output.className = 'result ' + (response.ok ? 'success' : 'error');
      } catch (err) {
        output.textContent = '请求失败: ' + err.message;
        output.className = 'result error';
      } finally {
        submitBtn.disabled = false;
        pollStatus();
      }
    });

    async function pollStatus() {
      refreshBtn.disabled = true;
      try {
        const response = await fetch('/api/status', { cache: 'no-store' });
        const data = await response.json();
        renderStatus(data);
      } catch (err) {
        setPill('rosPill', 'bad', '断开');
        setText('rosCli', err.message);
        setText('lastUpdate', '刷新失败');
      } finally {
        refreshBtn.disabled = false;
      }
    }

    function renderStatus(data) {
      setText('lastUpdate', new Date().toLocaleTimeString());
      setText('autoRefresh', `自动刷新 ${Number(data.poll_interval_sec || 0.5).toFixed(1)}s`);
      setPill('rosPill', data.ros.ok ? 'ok' : 'bad', data.ros.ok ? '在线' : '不可用');
      setText('rosCli', data.ros.message || '--');
      setText('topicCount', String(data.topics.length));

      const feedback = data.feedback;
      const feedbackOk = feedback.ok && feedback.fresh;
      const feedbackLabel = feedback.using_fallback ? '兜底' : (feedbackOk ? '在线' : (feedback.ok ? '过期' : '无反馈'));
      setPill('feedbackPill', feedbackOk && !feedback.using_fallback ? 'ok' : (feedback.ok ? 'warn' : 'bad'), feedbackLabel);
      const feedbackSource = feedback.source
        ? `${feedback.source}${feedback.using_fallback ? ' (/joint_states_feedback 无消息)' : ''}`
        : '--';
      setText('feedbackSource', feedbackSource);
      setText('feedbackAge', feedback.age_sec == null ? '--' : fmt(feedback.age_sec, 2, ' s'));
      setText('sampleAge', data.sample_age_sec == null ? '--' : fmt(data.sample_age_sec, 2, ' s'));

      const arm = data.arm_status;
      setPill('armPill', arm.ok ? (Number(arm.err_code || 0) === 0 ? 'ok' : 'warn') : 'bad', arm.ok ? '在线' : '无状态');
      setText('armStatus', arm.ok ? String(arm.arm_status ?? '--') : (arm.error || '--'));
      setText('motionStatus', arm.ok ? String(arm.motion_status ?? '--') : '--');
      setText('errCode', arm.ok ? String(arm.err_code ?? '--') : '--');

      const pose = data.link_pose;
      setPill('fkPill', pose.ok && pose.fresh ? 'ok' : (pose.ok ? 'warn' : 'bad'), pose.ok && pose.fresh ? '在线' : (pose.ok ? '过期' : '无 FK'));
      setText('fkFrame', pose.frame_id || '--');
      setText('fkAge', pose.age_sec == null ? '--' : fmt(pose.age_sec, 2, ' s'));
      const p = pose.pose || {};
      setText('px', fmt(p.x));
      setText('py', fmt(p.y));
      setText('pz', fmt(p.z));
      setText('pr', Number.isFinite(p.roll) ? p.roll.toFixed(1) + ' deg' : '--');
      setText('pp', Number.isFinite(p.pitch) ? p.pitch.toFixed(1) + ' deg' : '--');
      setText('pw', Number.isFinite(p.yaw) ? p.yaw.toFixed(1) + ' deg' : '--');

      const rows = (feedback.names || []).map((name, index) => {
        const rad = feedback.position && Number.isFinite(feedback.position[index]) ? feedback.position[index] : NaN;
        return `<tr><td>${escapeHtml(name)}</td><td>${fmt(rad, 4)}</td><td>${deg(rad)}</td></tr>`;
      }).join('');
      document.getElementById('jointRows').innerHTML = rows || '<tr><td colspan="3">暂无关节反馈</td></tr>';
      setText('previewSource', feedback.ok ? `来源: ${feedback.source}` : '等待关节 topic');
      setText('previewMeta', feedback.names && feedback.names.length ? `${feedback.names.length} joints` : '--');
      window.__lastJoints = feedback.position || [];
      window.__lastFeedback = feedback;
      drawPreview(feedback.position || [], feedback);

      const topicSet = new Set(data.topics || []);
      document.getElementById('topics').innerHTML = requiredTopics.map(topic =>
        `<span class="topic ${topicSet.has(topic) ? 'ok' : ''}">${topic}</span>`
      ).join('');
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
    }

    const preview3d = {
      ready: false,
      failed: false,
      container: null,
      renderer: null,
      scene: null,
      camera: null,
      armGroup: null,
      meshGroup: null,
      robotMeshes: {},
      meshesRequested: false,
      meshesLoaded: false,
      statusLabel: null,
      target: null,
      yaw: Math.PI,
      pitch: 0.48,
      distance: 2.4,
      viewSize: 1.05,
      zoomScale: 1,
      dragging: false,
      lastX: 0,
      lastY: 0,
    };

    function initPreview3D() {
      if (preview3d.ready || preview3d.failed) return preview3d.ready;
      const THREE = window.THREE;
      preview3d.container = document.getElementById('armPreview');
      if (!THREE || !preview3d.container) {
        preview3d.failed = true;
        if (preview3d.container) {
          preview3d.container.innerHTML = '<div style="padding:16px;color:#b91c1c">Three.js 未加载，无法显示 3D 视图。</div>';
        }
        return false;
      }

      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0xf8fafc);

      const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 20);
      const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.domElement.style.width = '100%';
      renderer.domElement.style.height = '100%';
      renderer.domElement.style.display = 'block';
      preview3d.container.appendChild(renderer.domElement);

      const hemi = new THREE.HemisphereLight(0xffffff, 0xcbd5e1, 1.8);
      scene.add(hemi);
      const key = new THREE.DirectionalLight(0xffffff, 2.2);
      key.position.set(0.5, 1.2, 0.7);
      scene.add(key);

      const grid = new THREE.GridHelper(1.2, 12, 0x94a3b8, 0xdbe3ee);
      grid.position.y = 0;
      scene.add(grid);

      const axes = new THREE.AxesHelper(0.45);
      scene.add(axes);

      const armGroup = new THREE.Group();
      scene.add(armGroup);
      const meshGroup = new THREE.Group();
      scene.add(meshGroup);

      preview3d.statusLabel = document.createElement('div');
      preview3d.statusLabel.style.cssText = 'position:absolute;left:10px;top:8px;right:10px;padding:4px 7px;border-radius:4px;background:rgba(255,255,255,.86);color:#344054;font:12px system-ui,sans-serif;line-height:1.35;pointer-events:none;white-space:normal;';
      preview3d.container.appendChild(preview3d.statusLabel);

      preview3d.container.addEventListener('pointerdown', (evt) => {
        preview3d.dragging = true;
        preview3d.lastX = evt.clientX;
        preview3d.lastY = evt.clientY;
        preview3d.container.classList.add('dragging');
        preview3d.container.setPointerCapture(evt.pointerId);
      });
      preview3d.container.addEventListener('pointermove', (evt) => {
        if (!preview3d.dragging) return;
        const dx = evt.clientX - preview3d.lastX;
        const dy = evt.clientY - preview3d.lastY;
        preview3d.lastX = evt.clientX;
        preview3d.lastY = evt.clientY;
        preview3d.yaw -= dx * 0.008;
        preview3d.pitch = Math.max(-0.05, Math.min(1.25, preview3d.pitch + dy * 0.006));
        renderPreview3D();
      });
      preview3d.container.addEventListener('pointerup', (evt) => {
        preview3d.dragging = false;
        preview3d.container.classList.remove('dragging');
        if (preview3d.container.hasPointerCapture(evt.pointerId)) preview3d.container.releasePointerCapture(evt.pointerId);
      });
      preview3d.container.addEventListener('pointercancel', () => {
        preview3d.dragging = false;
        preview3d.container.classList.remove('dragging');
      });
      preview3d.container.addEventListener('wheel', (evt) => {
        evt.preventDefault();
        preview3d.zoomScale = Math.max(0.45, Math.min(2.4, preview3d.zoomScale + evt.deltaY * 0.0015));
        resizePreview3D();
        renderPreview3D();
      }, { passive: false });

      preview3d.scene = scene;
      preview3d.camera = camera;
      preview3d.renderer = renderer;
      preview3d.armGroup = armGroup;
      preview3d.meshGroup = meshGroup;
      preview3d.target = new THREE.Vector3(0, 0.24, 0);
      preview3d.ready = true;
      loadUrdfMeshes();
      resizePreview3D();
      return true;
    }

    function resizePreview3D() {
      if (!preview3d.ready) return;
      const rect = preview3d.container.getBoundingClientRect();
      const width = Math.max(320, Math.round(rect.width));
      const height = Math.max(260, Math.round(rect.height));
      const aspect = width / height;
      const halfHeight = preview3d.viewSize * preview3d.zoomScale * 0.5;
      const halfWidth = halfHeight * aspect;
      preview3d.camera.left = -halfWidth;
      preview3d.camera.right = halfWidth;
      preview3d.camera.top = halfHeight;
      preview3d.camera.bottom = -halfHeight;
      preview3d.camera.updateProjectionMatrix();
      preview3d.renderer.setSize(width, height, false);
      renderPreview3D();
    }

    function updatePreviewCamera() {
      const target = preview3d.target || new THREE.Vector3(0, 0.24, 0);
      const radius = preview3d.distance;
      const x = target.x + Math.cos(preview3d.pitch) * Math.sin(preview3d.yaw) * radius;
      const y = target.y + Math.sin(preview3d.pitch) * radius;
      const z = target.z + Math.cos(preview3d.pitch) * Math.cos(preview3d.yaw) * radius;
      preview3d.camera.position.set(x, y, z);
      preview3d.camera.lookAt(target);
    }

    function renderPreview3D() {
      if (!preview3d.ready) return;
      updatePreviewCamera();
      preview3d.renderer.render(preview3d.scene, preview3d.camera);
    }

    function rosToPreviewPoint(ros) {
      return {
        x: ros.y,
        y: ros.z,
        z: ros.x,
        ros,
      };
    }

    const urdfJoints = [
      { name: 'joint1', xyz: [0, 0, 0.123], rpy: [0, 0, 0], axis: [0, 0, 1] },
      { name: 'joint2', xyz: [0, 0, 0], rpy: [1.5708, -0.10095, -3.1416], axis: [0, 0, 1] },
      { name: 'joint3', xyz: [0.28503, 0, 0], rpy: [0, 0, -1.759], axis: [0, 0, 1] },
      { name: 'joint4', xyz: [-0.021984, -0.25075, 0], rpy: [1.5708, 0, 0], axis: [0, 0, 1] },
      { name: 'joint5', xyz: [0, 0, 0], rpy: [-1.5708, 0, 0], axis: [0, 0, 1] },
      { name: 'joint6', xyz: [0.000088259, -0.091, 0], rpy: [1.5708, 0, 0], axis: [0, 0, 1] },
    ];
    const urdfGripperFingerOrigin = { xyz: [0, 0, 0.1358], rpy: [1.5708, 0, 0] };
    const urdfMeshSpecs = [
      { link: 'base_link', file: 'base_link.STL', color: 0xe5e7eb },
      { link: 'link1', file: 'link1.STL', color: 0xc9d2ee },
      { link: 'link2', file: 'link2.STL', color: 0xc9d2ee },
      { link: 'link3', file: 'link3.STL', color: 0xc9d2ee },
      { link: 'link4', file: 'link4.STL', color: 0xc9d2ee },
      { link: 'link5', file: 'link5.STL', color: 0xc9d2ee },
      { link: 'link6', file: 'link6.STL', color: 0xe5eaf0 },
      { link: 'gripper_base', file: 'gripper_base.STL', color: 0xc9d2ee },
      { link: 'link7', file: 'link7.STL', color: 0x9fb0d8 },
      { link: 'link8', file: 'link8.STL', color: 0x9fb0d8 },
    ];

    function matMul(a, b) {
      const out = new Array(16).fill(0);
      for (let row = 0; row < 4; row++) {
        for (let col = 0; col < 4; col++) {
          for (let k = 0; k < 4; k++) out[row * 4 + col] += a[row * 4 + k] * b[k * 4 + col];
        }
      }
      return out;
    }

    function matIdentity() {
      return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    }

    function matTranslate(x, y, z) {
      return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
    }

    function matRotX(angle) {
      const c = Math.cos(angle), s = Math.sin(angle);
      return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
    }

    function matRotY(angle) {
      const c = Math.cos(angle), s = Math.sin(angle);
      return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
    }

    function matRotZ(angle) {
      const c = Math.cos(angle), s = Math.sin(angle);
      return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    }

    function matRotAxis(axis, angle) {
      const [x0, y0, z0] = axis;
      const length = Math.hypot(x0, y0, z0) || 1;
      const x = x0 / length, y = y0 / length, z = z0 / length;
      const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
      return [
        t*x*x + c, t*x*y - s*z, t*x*z + s*y, 0,
        t*x*y + s*z, t*y*y + c, t*y*z - s*x, 0,
        t*x*z - s*y, t*y*z + s*x, t*z*z + c, 0,
        0, 0, 0, 1,
      ];
    }

    function matRpy(roll, pitch, yaw) {
      return matMul(matMul(matRotZ(yaw), matRotY(pitch)), matRotX(roll));
    }

    function transformPoint(m, point = [0, 0, 0]) {
      const [x, y, z] = point;
      return {
        x: m[0] * x + m[1] * y + m[2] * z + m[3],
        y: m[4] * x + m[5] * y + m[6] * z + m[7],
        z: m[8] * x + m[9] * y + m[10] * z + m[11],
      };
    }

    function matToThree(m) {
      const THREE = window.THREE;
      const out = new THREE.Matrix4();
      out.set(
        m[0], m[1], m[2], m[3],
        m[4], m[5], m[6], m[7],
        m[8], m[9], m[10], m[11],
        m[12], m[13], m[14], m[15]
      );
      return out;
    }

    function previewMatrixFromRos(m) {
      const THREE = window.THREE;
      const rosToPreview = new THREE.Matrix4().set(
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
      );
      const previewToRos = rosToPreview.clone().invert();
      return rosToPreview.clone().multiply(matToThree(m)).multiply(previewToRos);
    }

    function computeRobotState(joints) {
      const q = Array.from({length: 6}, (_, index) => Number.isFinite(joints[index]) ? joints[index] : 0);
      const gripperOpening = Number.isFinite(joints[6]) ? Math.max(0, Math.min(0.08, joints[6])) : 0;
      let transform = matIdentity();
      const rosPoints = [{x: 0, y: 0, z: 0}];
      const linkTransforms = { base_link: matIdentity() };

      for (let i = 0; i < urdfJoints.length; i++) {
        const joint = urdfJoints[i];
        const [x, y, z] = joint.xyz;
        const [roll, pitch, yaw] = joint.rpy;
        transform = matMul(transform, matMul(matTranslate(x, y, z), matRpy(roll, pitch, yaw)));
        transform = matMul(transform, matRotAxis(joint.axis, q[i]));
        linkTransforms[`link${i + 1}`] = transform;
        rosPoints.push(transformPoint(transform));
      }

      linkTransforms.gripper_base = transform;
      const [gx, gy, gz] = urdfGripperFingerOrigin.xyz;
      const [gr, gp, gw] = urdfGripperFingerOrigin.rpy;
      const fingerRoot = matMul(transform, matMul(matTranslate(gx, gy, gz), matRpy(gr, gp, gw)));
      rosPoints.push(transformPoint(fingerRoot));
      linkTransforms.link7 = matMul(fingerRoot, matTranslate(0, 0, gripperOpening * 0.5));
      linkTransforms.link8 = matMul(
        matMul(transform, matMul(matTranslate(gx, gy, gz), matRpy(gr, gp, -3.1416))),
        matTranslate(0, 0, -gripperOpening * 0.5)
      );

      const fingerLength = 0.0765;
      rosPoints.push(transformPoint(linkTransforms.link7, [0, -fingerLength, 0]));
      rosPoints.push(transformPoint(linkTransforms.link8, [0, -fingerLength, 0]));

      return {
        points: rosPoints.map(rosToPreviewPoint),
        linkTransforms,
      };
    }

    function makeRobotPoints(joints) {
      return computeRobotState(joints).points;
    }

    function applyRosLocalToPreviewGeometry(geometry) {
      const THREE = window.THREE;
      geometry.applyMatrix4(new THREE.Matrix4().set(
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
      ));
      geometry.computeVertexNormals();
      geometry.computeBoundingSphere();
      geometry.computeBoundingBox();
      return geometry;
    }

    function loadUrdfMeshes() {
      if (preview3d.meshesRequested) return;
      preview3d.meshesRequested = true;
      const THREE = window.THREE;
      if (!THREE || !THREE.STLLoader || !preview3d.meshGroup) {
        preview3d.statusLabel.textContent = 'STLLoader 未加载，使用简化预览';
        return;
      }

      const loader = new THREE.STLLoader();
      let loaded = 0;
      for (const spec of urdfMeshSpecs) {
        loader.load(
          `/urdf/meshes/${spec.file}`,
          (geometry) => {
            applyRosLocalToPreviewGeometry(geometry);
            const material = new THREE.MeshStandardMaterial({
              color: spec.color,
              roughness: 0.62,
              metalness: 0.08,
            });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.matrixAutoUpdate = false;
            mesh.castShadow = false;
            mesh.receiveShadow = true;
            preview3d.robotMeshes[spec.link] = mesh;
            preview3d.meshGroup.add(mesh);
            loaded += 1;
            preview3d.meshesLoaded = loaded === urdfMeshSpecs.length;
            updateUrdfMeshes(window.__lastJoints || []);
            renderPreview3D();
          },
          undefined,
          () => {
            loaded += 1;
            preview3d.meshesLoaded = false;
            updateUrdfMeshes(window.__lastJoints || []);
          }
        );
      }
    }

    function updateUrdfMeshes(joints) {
      if (!preview3d.meshGroup || !Object.keys(preview3d.robotMeshes).length) return false;
      const state = computeRobotState(joints);
      for (const [link, transform] of Object.entries(state.linkTransforms)) {
        const mesh = preview3d.robotMeshes[link];
        if (!mesh) continue;
        mesh.matrix.copy(previewMatrixFromRos(transform));
        mesh.matrixWorldNeedsUpdate = true;
        mesh.visible = true;
      }
      return preview3d.meshesLoaded;
    }

    function addCylinderBetween(group, a, b, radius, material) {
      const THREE = window.THREE;
      const start = new THREE.Vector3(a.x, a.y, a.z);
      const end = new THREE.Vector3(b.x, b.y, b.z);
      const direction = new THREE.Vector3().subVectors(end, start);
      const length = direction.length();
      if (length < 0.0001) return;
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 24), material);
      mesh.position.copy(start).add(end).multiplyScalar(0.5);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
      group.add(mesh);
    }

    function centerPreviewOnPoints(points) {
      const THREE = window.THREE;
      if (!points.length) return;

      let minX = points[0].x;
      let maxX = points[0].x;
      let minY = points[0].y;
      let maxY = points[0].y;
      let minZ = points[0].z;
      let maxZ = points[0].z;
      for (const point of points) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
        minZ = Math.min(minZ, point.z);
        maxZ = Math.max(maxZ, point.z);
      }

      const center = new THREE.Vector3(
        (minX + maxX) * 0.5,
        (minY + maxY) * 0.5,
        (minZ + maxZ) * 0.5
      );
      const spanX = maxX - minX;
      const spanY = maxY - minY;
      const spanZ = maxZ - minZ;
      const radius = Math.max(
        0.34,
        Math.sqrt(spanX * spanX + spanY * spanY + spanZ * spanZ) * 0.5
      );
      preview3d.target.copy(center);
      preview3d.viewSize = Math.max(1.05, radius * 3.35);
      preview3d.distance = Math.max(2.4, radius * 5.8);
      resizePreview3D();
    }

    function drawPreview(joints, feedback = {}) {
      if (!initPreview3D()) return;
      const THREE = window.THREE;
      const hasData = joints.some(Number.isFinite);
      const stale = feedback.ok && !feedback.fresh;
      const segmentMaterial = new THREE.MeshStandardMaterial({
        color: hasData ? (stale ? 0xb45309 : 0x1d4ed8) : 0x94a3b8,
        roughness: 0.55,
        metalness: 0.12,
      });
      const jointMaterial = new THREE.MeshStandardMaterial({ color: hasData ? 0x172033 : 0x64748b, roughness: 0.5 });
      const tipMaterial = new THREE.MeshStandardMaterial({ color: 0x15803d, roughness: 0.45 });
      const shadowMaterial = new THREE.MeshBasicMaterial({ color: 0xcbd5e1, transparent: true, opacity: 0.42 });

      preview3d.armGroup.clear();
      const state = computeRobotState(joints);
      const points = state.points;
      const usingMeshes = updateUrdfMeshes(joints);
      preview3d.armGroup.visible = !usingMeshes;
      centerPreviewOnPoints(points);
      const shadow = points.map(point => ({x: point.x, y: 0.006, z: point.z}));
      const segmentPairs = [];
      for (let i = 0; i < Math.min(7, points.length - 1); i++) segmentPairs.push([i, i + 1]);
      if (points.length >= 10) {
        segmentPairs.push([7, 8], [7, 9]);
      } else if (points.length > 7) {
        for (let i = 7; i < points.length - 1; i++) segmentPairs.push([i, i + 1]);
      }

      for (const [a, b] of segmentPairs) {
        addCylinderBetween(preview3d.armGroup, shadow[a], shadow[b], 0.008, shadowMaterial);
      }
      for (const [a, b] of segmentPairs) {
        addCylinderBetween(preview3d.armGroup, points[a], points[b], a < 2 ? 0.028 : 0.022, segmentMaterial);
      }
      for (let i = 0; i < points.length; i++) {
        const geometry = new THREE.SphereGeometry(i === points.length - 1 ? 0.042 : 0.032, 24, 16);
        const mesh = new THREE.Mesh(geometry, i === points.length - 1 ? tipMaterial : jointMaterial);
        mesh.position.set(points[i].x, points[i].y, points[i].z);
        preview3d.armGroup.add(mesh);
      }

      const tip = points.length >= 10
        ? {
            x: (points[8].x + points[9].x) * 0.5,
            y: (points[8].y + points[9].y) * 0.5,
            z: (points[8].z + points[9].z) * 0.5,
            ros: {
              x: (points[8].ros.x + points[9].ros.x) * 0.5,
              y: (points[8].ros.y + points[9].ros.y) * 0.5,
              z: (points[8].ros.z + points[9].ros.z) * 0.5,
            },
          }
        : points[points.length - 1];
      const rosTip = tip.ros || {x: tip.z, y: tip.x, z: tip.y};
      preview3d.statusLabel.textContent = hasData
        ? `${usingMeshes ? 'URDF STL' : 'URDF FK'} · ${feedback.source || 'joint topic'} ${stale ? 'stale' : 'live'} · ROS tip x ${fmt(rosTip.x, 3)} y ${fmt(rosTip.y, 3)} z ${fmt(rosTip.z, 3)}`
        : (usingMeshes ? 'URDF STL 已加载，等待关节反馈' : '等待 /joint_states_feedback 或 /joint_states');
      renderPreview3D();
    }

    xyPad.addEventListener('pointerdown', (evt) => {
      xyState.dragging = true;
      xyPad.setPointerCapture(evt.pointerId);
      setXYFromEvent(evt);
    });
    xyPad.addEventListener('pointermove', (evt) => {
      if (xyState.dragging) setXYFromEvent(evt);
    });
    xyPad.addEventListener('pointerup', (evt) => {
      xyState.dragging = false;
      if (xyPad.hasPointerCapture(evt.pointerId)) xyPad.releasePointerCapture(evt.pointerId);
    });
    xyPad.addEventListener('pointercancel', () => { xyState.dragging = false; });
    zRange.addEventListener('input', () => {
      zInput.value = Number(zRange.value).toFixed(3);
      drawXYPad();
    });
    for (const input of [xInput, yInput, zInput]) {
      input.addEventListener('input', () => {
        if (input === zInput && Number.isFinite(Number(zInput.value))) {
          zRange.value = Math.max(Number(zRange.min), Math.min(Number(zRange.max), Number(zInput.value))).toFixed(3);
        }
        drawXYPad();
      });
    }
    useCurrentBtn.addEventListener('click', async () => {
      try {
        await loadCurrentPose({ center: false });
      } catch (err) {
        output.style.display = 'block';
        output.className = 'result error';
        output.textContent = '读取当前位姿失败: ' + err.message;
      }
    });
    centerCurrentBtn.addEventListener('click', async () => {
      try {
        await loadCurrentPose({ center: true });
      } catch (err) {
        output.style.display = 'block';
        output.className = 'result error';
        output.textContent = '当前点居中失败: ' + err.message;
      }
    });
    centerOriginBtn.addEventListener('click', () => setXYCenter(0.0, 0.0));
    positionOnlyInput.addEventListener('change', updateOrientationMode);
    for (const button of document.querySelectorAll('.jog-button')) {
      button.addEventListener('pointerdown', (evt) => {
        evt.preventDefault();
        button.setPointerCapture(evt.pointerId);
        startJog(button.dataset.jogAxis, Number(button.dataset.jogDir), evt.pointerId);
      });
      button.addEventListener('pointerup', (evt) => {
        if (button.hasPointerCapture(evt.pointerId)) button.releasePointerCapture(evt.pointerId);
        stopJog();
      });
      button.addEventListener('pointercancel', stopJog);
      button.addEventListener('pointerleave', () => {
        if (jogState.activePointer != null) stopJog();
      });
    }

    const jogKeyMap = new Map([
      ['KeyA', ['x', -1]], ['KeyD', ['x', 1]],
      ['KeyS', ['y', -1]], ['KeyW', ['y', 1]],
      ['KeyQ', ['z', -1]], ['KeyE', ['z', 1]],
      ['KeyI', ['roll', -1]], ['KeyK', ['roll', 1]],
      ['KeyJ', ['pitch', -1]], ['KeyL', ['pitch', 1]],
      ['KeyU', ['yaw', -1]], ['KeyO', ['yaw', 1]],
    ]);
    window.addEventListener('keydown', (evt) => {
      if (evt.repeat || evt.target instanceof HTMLInputElement) return;
      const mapping = jogKeyMap.get(evt.code);
      if (!mapping) return;
      evt.preventDefault();
      jogState.activeKeys.set(evt.code, mapping);
      startJog(mapping[0], mapping[1]);
    });
    window.addEventListener('keyup', (evt) => {
      if (!jogState.activeKeys.has(evt.code)) return;
      jogState.activeKeys.delete(evt.code);
      if (jogState.activeKeys.size === 0) {
        stopJog();
        return;
      }
      const activeMappings = Array.from(jogState.activeKeys.values());
      const next = activeMappings[activeMappings.length - 1];
      startJog(next[0], next[1]);
    });
    refreshBtn.addEventListener('click', pollStatus);
    window.addEventListener('resize', () => {
      drawXYPad();
      drawPreview(window.__lastJoints || [], window.__lastFeedback || {});
    });
    updateOrientationMode();
    drawXYPad();
    setInterval(pollStatus, 500);
    pollStatus();
  </script>
</body>
</html>
"""


def _planner_script_path() -> Path:
    """Return the path to the pose planner script next to this file."""
    return Path(__file__).resolve().with_name("piper_moveit_pose_planner.py")


def _run_planner(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    planner: str,
    planning_time: float,
    velocity_scaling: float,
    position_only: bool,
) -> subprocess.CompletedProcess:
    """Run the pose planner subprocess and return the completed process."""
    script = _planner_script_path()
    if not script.exists():
        raise FileNotFoundError(f"Planner script not found: {script}")

    cmd = [
        "python3",
        str(script),
        "--x", str(x),
        "--y", str(y),
        "--z", str(z),
        "--roll", str(roll),
        "--pitch", str(pitch),
        "--yaw", str(yaw),
        "--planning-time", str(planning_time),
        "--velocity-scaling", str(velocity_scaling),
    ]
    if planner:
        cmd.extend(["--planner", planner])
    if position_only:
        cmd.append("--position-only")
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )


def _parse_form(body: bytes, content_type: str) -> dict[str, str]:
    """Parse the small HTML form without relying on the removed cgi module."""
    if "multipart/form-data" in content_type:
        boundary_match = re.search(r"boundary=([^;]+)", content_type)
        if not boundary_match:
            return {}

        boundary = boundary_match.group(1).strip().strip('"')
        marker = ("--" + boundary).encode("utf-8")
        values: dict[str, str] = {}
        for part in body.split(marker):
            if b"Content-Disposition:" not in part:
                continue

            header, _, payload = part.partition(b"\r\n\r\n")
            name_match = re.search(rb'name="([^"]+)"', header)
            if not name_match:
                continue

            name = name_match.group(1).decode("utf-8", errors="replace")
            value = payload.strip().removesuffix(b"--").strip()
            values[name] = value.decode("utf-8", errors="replace")
        return values

    parsed = parse_qs(body.decode("utf-8", errors="replace"), keep_blank_values=True)
    return {key: values[-1] if values else "" for key, values in parsed.items()}


def _parse_float(form: dict[str, str], key: str, default: float) -> float:
    """Parse a float value from the form, falling back to a default."""
    value = form.get(key)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{key} 必须是数字") from exc


def _run_topic_cli(args: list[str], timeout: float = 2.0) -> subprocess.CompletedProcess:
    """Run ros2 topic or ROS1 rostopic commands with a short timeout."""
    cli = os.environ.get("WEB_POSE_TOPIC_CLI", "").strip().lower()
    if not cli:
        cli = "rostopic" if os.environ.get("ROS_VERSION") == "1" else "ros2"

    if cli == "rostopic":
        command = ["rostopic", *args]
    else:
        command = ["ros2", "topic", *args]

    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


ROS_ECHO_TIMEOUT = float(os.environ.get("WEB_POSE_ROS_ECHO_TIMEOUT", "2.0"))
CANONICAL_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
URDF_MESH_DIR = Path(__file__).resolve().parent.parent / "Assets" / "URDF" / "piper_description" / "meshes"
URDF_MESH_FILES = {
    "base_link.STL",
    "link1.STL",
    "link2.STL",
    "link3.STL",
    "link4.STL",
    "link5.STL",
    "link6.STL",
    "gripper_base.STL",
    "link7.STL",
    "link8.STL",
}


def _topic_list() -> tuple[list[str], str]:
    try:
        result = _run_topic_cli(["list"], timeout=0.8)
    except FileNotFoundError:
        return [], "topic command not found"
    except subprocess.TimeoutExpired:
        return [], "topic list timeout"

    if result.returncode != 0:
        message = (result.stderr or result.stdout or "topic list failed").strip()
        return [], message

    topics = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return topics, "topic list ok"


def _echo_topic(topic: str, field: str | None = None, timeout: float = ROS_ECHO_TIMEOUT) -> tuple[bool, str, str]:
    cli = os.environ.get("WEB_POSE_TOPIC_CLI", "").strip().lower()
    if not cli:
        cli = "rostopic" if os.environ.get("ROS_VERSION") == "1" else "ros2"

    if cli == "rostopic":
        args = ["echo", "-n", "1", topic]
        if field:
            args.append(field)
    else:
        args = ["echo", topic, "--once"]
        if field:
            args.extend(["--field", field])

    try:
        result = _run_topic_cli(args, timeout=timeout)
    except FileNotFoundError:
        return False, "", "topic command not found"
    except subprocess.TimeoutExpired:
        return False, "", f"{topic} timeout"

    if result.returncode != 0 or not result.stdout.strip():
        message = (result.stderr or result.stdout or f"{topic} unavailable").strip()
        return False, result.stdout, message

    return True, result.stdout, ""


def _extract_scalar(text: str, key: str) -> str | None:
    match = re.search(rf"(?m)^\s*{re.escape(key)}:\s*(.+?)\s*$", text)
    if not match:
        return None
    value = match.group(1).strip()
    if (value.startswith("'") and value.endswith("'")) or (value.startswith('"') and value.endswith('"')):
        return value[1:-1]
    return value


def _extract_number(text: str, key: str) -> float | None:
    value = _extract_scalar(text, key)
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _extract_int(text: str, key: str) -> int | None:
    value = _extract_scalar(text, key)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        try:
            return int(float(value))
        except ValueError:
            return None


def _extract_list(text: str, key: str) -> list[str | float]:
    match = re.search(rf"(?ms)^\s*{re.escape(key)}:\s*\n((?:\s*-\s*.+\n?)+)", text)
    if not match:
        inline = _extract_scalar(text, key)
        if inline and inline.startswith("[") and inline.endswith("]"):
            return [item.strip().strip("'\"") for item in inline[1:-1].split(",") if item.strip()]
        return []

    values: list[str | float] = []
    for raw in re.findall(r"(?m)^\s*-\s*(.+?)\s*$", match.group(1)):
        item = raw.strip().strip("'\"")
        try:
            values.append(float(item))
        except ValueError:
            values.append(item)
    return values


def _canonical_joint_name(name: object) -> str:
    value = str(name).strip()
    return "gripper" if value == "joint7" else value


def _normalize_joint_state(names: list[str], positions: list[float]) -> tuple[list[str], list[float]]:
    by_name: dict[str, float] = {}
    for index, position in enumerate(positions):
        if index < len(names):
            name = _canonical_joint_name(names[index])
        elif index < len(CANONICAL_JOINT_NAMES):
            name = CANONICAL_JOINT_NAMES[index]
        else:
            name = f"joint_{index + 1}"
        by_name[name] = float(position)

    normalized_names: list[str] = []
    normalized_positions: list[float] = []
    for name in CANONICAL_JOINT_NAMES:
        if name in by_name:
            normalized_names.append(name)
            normalized_positions.append(by_name[name])

    for name, position in by_name.items():
        if name not in CANONICAL_JOINT_NAMES:
            normalized_names.append(name)
            normalized_positions.append(position)

    return normalized_names, normalized_positions


def _header_age(text: str) -> float | None:
    sec = _extract_number(text, "sec")
    nanosec = _extract_number(text, "nanosec")
    if sec is None:
        return None

    stamp = sec + (nanosec or 0.0) * 1e-9
    if stamp <= 0.0:
        return None

    age = time.time() - stamp
    if age < -3600.0:
        return None
    return max(0.0, age)


def _pose_from_echo(text: str) -> dict[str, object]:
    x = y = z = None
    roll = pitch = yaw = None

    position_match = re.search(
        r"(?ms)position:\s*\n\s*x:\s*([-+0-9.eE]+)\s*\n\s*y:\s*([-+0-9.eE]+)\s*\n\s*z:\s*([-+0-9.eE]+)",
        text,
    )
    orientation_match = re.search(
        r"(?ms)orientation:\s*\n\s*x:\s*([-+0-9.eE]+)\s*\n\s*y:\s*([-+0-9.eE]+)\s*\n\s*z:\s*([-+0-9.eE]+)",
        text,
    )
    if position_match:
        x, y, z = (float(position_match.group(i)) for i in range(1, 4))
    if orientation_match:
        roll, pitch, yaw = (float(orientation_match.group(i)) for i in range(1, 4))

    return {
        "x": x,
        "y": y,
        "z": z,
        "roll": None if roll is None else roll * 180.0 / 3.141592653589793,
        "pitch": None if pitch is None else pitch * 180.0 / 3.141592653589793,
        "yaw": None if yaw is None else yaw * 180.0 / 3.141592653589793,
    }


def _stamp_age(stamp) -> float | None:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None:
        return None

    stamp_sec = float(sec) + float(nanosec or 0.0) * 1e-9
    if stamp_sec <= 0.0:
        return None

    age = time.time() - stamp_sec
    if age < -3600.0:
        return None
    if age > 86400.0:
        return None
    return max(0.0, age)


class RosStatusMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.topics: list[str] = []
        self.feedback: dict[str, object] = {"ok": False, "fresh": False, "source": "/joint_states_feedback", "error": "waiting"}
        self.fallback_feedback: dict[str, object] = {"ok": False, "fresh": False, "source": "/joint_states", "error": "waiting"}
        self.arm_status: dict[str, object] = {"ok": False, "error": "waiting"}
        self.link_pose: dict[str, object] = {"ok": False, "fresh": False, "error": "waiting"}
        self.ros_ok = False
        self.message = "starting rclpy monitor"

    def run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from piper_msgs.msg import PiperStatusMsg
            from sensor_msgs.msg import JointState
        except Exception as exc:  # noqa: BLE001
            with self.lock:
                self.message = f"rclpy monitor unavailable: {exc}"
            return

        try:
            rclpy.init(args=None)
            node = rclpy.create_node("piper_web_status_monitor")
            node.create_subscription(JointState, "/joint_states_feedback", lambda msg: self._set_joint_state("/joint_states_feedback", msg), 10)
            node.create_subscription(JointState, "/joint_states", lambda msg: self._set_joint_state("/joint_states", msg), 10)
            node.create_subscription(PiperStatusMsg, "/arm_status", self._set_arm_status, 10)
            node.create_subscription(PoseStamped, "/link6_pose", self._set_link_pose, 10)

            while rclpy.ok():
                with self.lock:
                    self.topics = [name for name, _ in node.get_topic_names_and_types()]
                    self.ros_ok = True
                    self.message = "rclpy monitor ok"
                    self._refresh_fresh_flags()
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:  # noqa: BLE001
            with self.lock:
                self.ros_ok = False
                self.message = f"rclpy monitor failed: {exc}"

    def payload(self) -> dict[str, object]:
        with self.lock:
            feedback = dict(self.feedback)
            fallback = dict(self.fallback_feedback)
            has_preferred_topic = "/joint_states_feedback" in self.topics
            if not feedback.get("ok") and fallback.get("ok") and not has_preferred_topic:
                preferred_error = feedback.get("error")
                feedback = fallback
                feedback["preferred_source"] = "/joint_states_feedback"
                feedback["preferred_error"] = preferred_error
                feedback["using_fallback"] = True
            else:
                feedback["preferred_source"] = "/joint_states_feedback"
                feedback["preferred_error"] = feedback.get("error") if not feedback.get("ok") else None
                feedback["using_fallback"] = False

            return {
                "ros": {"ok": self.ros_ok, "message": self.message},
                "topics": list(self.topics),
                "link_pose": dict(self.link_pose),
                "feedback": feedback,
                "arm_status": dict(self.arm_status),
            }

    def _set_joint_state(self, source: str, msg) -> None:
        names, positions = _normalize_joint_state(
            [str(value) for value in msg.name],
            [float(value) for value in msg.position],
        )
        data = {
            "ok": True,
            "fresh": True,
            "source": source,
            "age_sec": _stamp_age(msg.header.stamp),
            "names": names,
            "position": positions,
            "received_at": time.time(),
        }
        with self.lock:
            if source == "/joint_states_feedback":
                self.feedback = data
            else:
                self.fallback_feedback = data

    def _set_arm_status(self, msg) -> None:
        with self.lock:
            self.arm_status = {
                "ok": True,
                "ctrl_mode": int(msg.ctrl_mode),
                "arm_status": int(msg.arm_status),
                "mode_feedback": int(msg.mode_feedback),
                "teach_status": int(msg.teach_status),
                "motion_status": int(msg.motion_status),
                "trajectory_num": int(msg.trajectory_num),
                "err_code": int(msg.err_code),
                "communication_ok": [
                    bool(msg.communication_status_joint_1),
                    bool(msg.communication_status_joint_2),
                    bool(msg.communication_status_joint_3),
                    bool(msg.communication_status_joint_4),
                    bool(msg.communication_status_joint_5),
                    bool(msg.communication_status_joint_6),
                ],
                "joint_angle_limit": [
                    bool(msg.joint_1_angle_limit),
                    bool(msg.joint_2_angle_limit),
                    bool(msg.joint_3_angle_limit),
                    bool(msg.joint_4_angle_limit),
                    bool(msg.joint_5_angle_limit),
                    bool(msg.joint_6_angle_limit),
                ],
                "received_at": time.time(),
            }

    def _set_link_pose(self, msg) -> None:
        with self.lock:
            self.link_pose = {
                "ok": True,
                "fresh": True,
                "age_sec": _stamp_age(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "pose": {
                    "x": float(msg.pose.position.x),
                    "y": float(msg.pose.position.y),
                    "z": float(msg.pose.position.z),
                    "roll": float(msg.pose.orientation.x) * 180.0 / 3.141592653589793,
                    "pitch": float(msg.pose.orientation.y) * 180.0 / 3.141592653589793,
                    "yaw": float(msg.pose.orientation.z) * 180.0 / 3.141592653589793,
                },
                "received_at": time.time(),
            }

    def _refresh_fresh_flags(self) -> None:
        now = time.time()
        for state in (self.feedback, self.fallback_feedback, self.link_pose):
            if state.get("ok") and state.get("received_at") is not None:
                state["fresh"] = now - float(state["received_at"]) < 2.5


def _link_pose_status() -> dict[str, object]:
    ok, stdout, error = _echo_topic("/link6_pose")
    if not ok:
        return {"ok": False, "fresh": False, "error": error}

    age = _header_age(stdout)
    return {
        "ok": True,
        "fresh": age is None or age < 2.5,
        "age_sec": age,
        "frame_id": _extract_scalar(stdout, "frame_id"),
        "pose": _pose_from_echo(stdout),
    }


def _joint_state_status(topics: list[str]) -> dict[str, object]:
    preferred_source = "/joint_states_feedback"
    fallback_source = "/joint_states"
    has_preferred_topic = preferred_source in topics
    source = preferred_source if has_preferred_topic else fallback_source
    if source not in topics:
        return {"ok": False, "fresh": False, "source": source, "error": f"missing {source}"}

    ok, stdout, error = _echo_topic(source)
    preferred_error = error if source == preferred_source and not ok else None
    using_fallback = False

    if not ok:
        return {
            "ok": False,
            "fresh": False,
            "source": source,
            "error": error,
            "preferred_source": preferred_source,
            "preferred_error": preferred_error,
            "using_fallback": using_fallback,
        }

    names = [str(value) for value in _extract_list(stdout, "name")]
    positions = [float(value) for value in _extract_list(stdout, "position") if isinstance(value, float)]
    names, positions = _normalize_joint_state(names, positions)
    age = _header_age(stdout)
    return {
        "ok": True,
        "fresh": age is None or age < 2.5,
        "source": source,
        "age_sec": age,
        "names": names,
        "position": positions,
        "preferred_source": preferred_source,
        "preferred_error": preferred_error,
        "using_fallback": using_fallback,
    }


def _arm_status() -> dict[str, object]:
    ok, stdout, error = _echo_topic("/arm_status")
    if not ok:
        return {"ok": False, "error": error}

    comms = [
        bool(_extract_scalar(stdout, f"communication_status_joint_{index}") == "true")
        for index in range(1, 7)
    ]
    limits = [
        bool(_extract_scalar(stdout, f"joint_{index}_angle_limit") == "true")
        for index in range(1, 7)
    ]
    return {
        "ok": True,
        "ctrl_mode": _extract_int(stdout, "ctrl_mode"),
        "arm_status": _extract_int(stdout, "arm_status"),
        "mode_feedback": _extract_int(stdout, "mode_feedback"),
        "teach_status": _extract_int(stdout, "teach_status"),
        "motion_status": _extract_int(stdout, "motion_status"),
        "trajectory_num": _extract_int(stdout, "trajectory_num"),
        "err_code": _extract_int(stdout, "err_code"),
        "communication_ok": comms,
        "joint_angle_limit": limits,
    }


def _status_payload() -> dict[str, object]:
    if ROS_MONITOR is not None:
        return ROS_MONITOR.payload()

    topics, message = _topic_list()
    ros_ok = bool(topics)
    return {
        "ros": {"ok": ros_ok, "message": message},
        "topics": topics,
        "link_pose": _link_pose_status() if ros_ok and "/link6_pose" in topics else {"ok": False, "fresh": False, "error": "missing /link6_pose"},
        "feedback": _joint_state_status(topics) if ros_ok else {"ok": False, "fresh": False, "error": message},
        "arm_status": _arm_status() if ros_ok and "/arm_status" in topics else {"ok": False, "error": "missing /arm_status"},
    }


def _sample_status_loop() -> None:
    while True:
        try:
            payload = _status_payload()
            payload["ok"] = True
            payload["updated_at"] = time.time()
            payload["poll_interval_sec"] = STATUS_POLL_INTERVAL
        except Exception as exc:  # noqa: BLE001
            payload = {
                "ok": False,
                "updated_at": time.time(),
                "poll_interval_sec": STATUS_POLL_INTERVAL,
                "ros": {"ok": False, "message": str(exc)},
                "topics": [],
                "link_pose": {"ok": False, "fresh": False, "error": str(exc)},
                "feedback": {"ok": False, "fresh": False, "error": str(exc)},
                "arm_status": {"ok": False, "error": str(exc)},
            }

        with STATUS_CACHE_LOCK:
            STATUS_CACHE.clear()
            STATUS_CACHE.update(payload)

        time.sleep(max(0.1, STATUS_POLL_INTERVAL))


def _cached_status_payload() -> dict[str, object]:
    with STATUS_CACHE_LOCK:
        payload = dict(STATUS_CACHE)

    updated_at = float(payload.get("updated_at") or 0.0)
    payload["sample_age_sec"] = None if updated_at <= 0.0 else max(0.0, time.time() - updated_at)
    payload.setdefault("poll_interval_sec", STATUS_POLL_INTERVAL)
    payload.setdefault("ros", {"ok": False, "message": payload.get("error", "status unavailable")})
    payload.setdefault("topics", [])
    payload.setdefault("link_pose", {"ok": False, "fresh": False, "error": "status unavailable"})
    payload.setdefault("feedback", {"ok": False, "fresh": False, "error": "status unavailable"})
    payload.setdefault("arm_status", {"ok": False, "error": "status unavailable"})
    return payload


class ReusableHTTPServer(ThreadingHTTPServer):
    """HTTPServer subclass that allows quick restart on the same address."""

    allow_reuse_address = True


class PoseControlHandler(BaseHTTPRequestHandler):
    """HTTP request handler for the pose control web interface."""

    def handle(self) -> None:
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format: str, *args) -> None:  # noqa: ANN002
        """Suppress default access logs; rely on application logging."""
        pass

    def _send_response(self, body: str, status: int = 200) -> None:
        try:
            self.send_response(status)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, payload: dict[str, object], status: int = 200) -> None:
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_file(self, path: Path, content_type: str) -> None:
        try:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(path.stat().st_size))
            self.send_header("Cache-Control", "public, max-age=3600")
            self.end_headers()
            with path.open("rb") as file:
                while True:
                    chunk = file.read(1024 * 128)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send_response(HTML_PAGE)
            return
        if path == "/api/status":
            self._send_json(_cached_status_payload())
            return
        if path == "/current_pose":
            self._send_current_pose()
            return
        if path.startswith("/urdf/meshes/"):
            self._send_urdf_mesh(path)
            return
        self._send_response("<h1>404 Not Found</h1>", 404)

    def _send_urdf_mesh(self, path: str) -> None:
        filename = Path(path).name
        if filename not in URDF_MESH_FILES:
            self._send_response("<h1>404 Not Found</h1>", 404)
            return

        mesh_path = URDF_MESH_DIR / filename
        if not mesh_path.exists() or not mesh_path.is_file():
            self._send_response("<h1>404 Not Found</h1>", 404)
            return

        self._send_file(mesh_path, "model/stl")

    def _send_current_pose(self) -> None:
        """Return the current link6 pose as JSON via ros2 topic echo."""
        payload = _link_pose_status()
        if not payload.get("ok"):
            self._send_json({"error": payload.get("error", "FK data not available")}, 503)
            return

        pose = payload.get("pose") or {}
        self._send_json(pose)

    def _handle_jog(self) -> None:
        content_length = self.headers.get("Content-Length")
        if content_length is None:
            self._send_json({"ok": False, "error": "missing request body"}, 400)
            return

        if not JOG_LOCK.acquire(blocking=False):
            self._send_json({"ok": False, "error": "jog planner is busy"}, 429)
            return

        try:
            length = int(content_length)
            body = self.rfile.read(length)
            form = _parse_form(body, self.headers.get("Content-Type", ""))
            x = _parse_float(form, "x", 0.3)
            y = _parse_float(form, "y", 0.0)
            z = _parse_float(form, "z", 0.4)
            roll = _parse_float(form, "roll", 0.0)
            pitch = _parse_float(form, "pitch", 0.0)
            yaw = _parse_float(form, "yaw", 0.0)
            planner = (form.get("planner") or "").strip()
            planning_time = _parse_float(form, "planning_time", 1.0)
            velocity_scaling = _parse_float(form, "velocity_scaling", 0.3)
            position_only = form.get("position_only") == "1"

            result = _run_planner(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                planner=planner,
                planning_time=planning_time,
                velocity_scaling=velocity_scaling,
                position_only=position_only,
            )
        except ValueError as exc:
            self._send_json({"ok": False, "error": f"invalid jog parameter: {exc}"}, 400)
            return
        except subprocess.TimeoutExpired:
            self._send_json({"ok": False, "error": "jog planning timed out"}, 500)
            return
        except Exception as exc:  # noqa: BLE001
            self._send_json({"ok": False, "error": str(exc)}, 500)
            return
        finally:
            JOG_LOCK.release()

        ok = result.returncode == 0
        self._send_json(
            {
                "ok": ok,
                "exit_code": result.returncode,
                "stdout": result.stdout[-3000:],
                "stderr": result.stderr[-3000:],
            },
            200 if ok else 500,
        )

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/jog":
            self._handle_jog()
            return
        if path != "/move":
            self._send_response("<h1>404 Not Found</h1>", 404)
            return

        content_length = self.headers.get("Content-Length")
        if content_length is None:
            self._send_response("缺少请求体", 400)
            return

        try:
            length = int(content_length)
            body = self.rfile.read(length)
            form = _parse_form(body, self.headers.get("Content-Type", ""))

            x = _parse_float(form, "x", 0.3)
            y = _parse_float(form, "y", 0.0)
            z = _parse_float(form, "z", 0.4)
            roll = _parse_float(form, "roll", 0.0)
            pitch = _parse_float(form, "pitch", 0.0)
            yaw = _parse_float(form, "yaw", 0.0)
            planner = (form.get("planner") or "").strip()
            planning_time = _parse_float(form, "planning_time", 5.0)
            velocity_scaling = _parse_float(form, "velocity_scaling", 0.3)
            position_only = form.get("position_only") == "1"
        except ValueError as exc:
            self._send_response(f"<div class='error'>参数错误: {html.escape(str(exc))}</div>", 400)
            return
        except Exception as exc:  # noqa: BLE001
            self._send_response(f"<div class='error'>请求解析失败: {html.escape(str(exc))}</div>", 400)
            return

        try:
            result = _run_planner(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                planner=planner,
                planning_time=planning_time,
                velocity_scaling=velocity_scaling,
                position_only=position_only,
            )
        except FileNotFoundError as exc:
            self._send_response(
                f"<div class='error'>找不到规划脚本: {html.escape(str(exc))}</div>", 500
            )
            return
        except subprocess.TimeoutExpired:
            self._send_response("<div class='error'>规划执行超时</div>", 500)
            return
        except Exception as exc:  # noqa: BLE001
            self._send_response(
                f"<div class='error'>执行失败: {html.escape(str(exc))}</div>", 500
            )
            return

        stdout = html.escape(result.stdout)
        stderr = html.escape(result.stderr)
        if result.returncode == 0:
            body = f"<div class='success'>执行成功 (exit {result.returncode})</div>\n<pre>{stdout}</pre>"
        else:
            body = (
                f"<div class='error'>执行失败 (exit {result.returncode})</div>\n"
                f"<pre>{stderr}\n{stdout}</pre>"
            )
        self._send_response(body, 200 if result.returncode == 0 else 500)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Web interface for PiPER arm MoveIt pose planning."
    )
    parser.add_argument(
        "--host",
        type=str,
        default=os.environ.get("WEB_POSE_HOST", DEFAULT_HOST),
        help=f"Bind address (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("WEB_POSE_PORT", DEFAULT_PORT)),
        help=f"Bind port (default: {DEFAULT_PORT})",
    )
    return parser


def main() -> int:
    global ROS_MONITOR

    parser = build_parser()
    args = parser.parse_args()

    try:
        server = ReusableHTTPServer((args.host, args.port), PoseControlHandler)
    except OSError as exc:
        print(
            f"无法绑定到 {args.host}:{args.port}: {exc}",
            file=sys.stderr,
        )
        print(
            "提示: 该端口可能已被占用。可通过环境变量或参数更换端口，例如:",
            file=sys.stderr,
        )
        print(
            f"  WEB_POSE_PORT=8081 python3 {__file__}",
            file=sys.stderr,
        )
        return 1

    ROS_MONITOR = RosStatusMonitor()
    threading.Thread(target=ROS_MONITOR.run, daemon=True).start()

    print(f"PiPER web pose server running at http://{args.host}:{args.port}")
    threading.Thread(target=_sample_status_loop, daemon=True).start()
    print(f"Sampling ROS status every {STATUS_POLL_INTERVAL:.2f}s.")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
