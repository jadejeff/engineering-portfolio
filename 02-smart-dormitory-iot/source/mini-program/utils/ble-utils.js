/**
 * BLE 工具函数 — 集中管理 ArrayBuffer 转换、上下文获取等公共逻辑
 */

/**
 * ArrayBuffer → UTF-8 字符串（自动过滤末尾 0x00）
 */
function buf2Str(buf) {
  try {
    const arr = new Uint8Array(buf);
    let end = arr.length;
    while (end > 0 && arr[end - 1] === 0) end--;
    let str = '';
    for (let i = 0; i < end; i++) {
      str += String.fromCharCode(arr[i]);
    }
    return str;
  } catch (e) {
    return '';
  }
}

/**
 * ArrayBuffer → 十六进制字符串（调试用）
 */
function buf2Hex(buf) {
  try {
    const arr = new Uint8Array(buf);
    let str = '';
    for (let i = 0; i < arr.length; i++) {
      str += arr[i].toString(16).padStart(2, '0') + ' ';
    }
    return str.trimEnd().toUpperCase();
  } catch (e) {
    return '';
  }
}

/**
 * 从 options + globalData 获取 BLE 连接参数
 * @param {Object} options - 页面 onLoad 的 options
 * @param {Array<string>} keys  - 需要的参数 keys，如 ['deviceId','serviceId','writeCharId']
 * @returns {Object} { deviceId, serviceId, ... }
 */
function getBLEParams(options, keys) {
  const ctx = getApp().globalData.bleContext || {};
  const result = {};
  keys.forEach(key => {
    result[key] = options[key] || ctx[key] || '';
  });
  return result;
}

/**
 * 构建发送缓冲区
 * @param {number|string|Array<number>} data
 *   - number: 单字节（旧兼容）
 *   - Array<number>: 多字节命令（新协议），如 [0xA0, 0x01]
 *   - string: ASCII 字符串
 * @returns {ArrayBuffer}
 */
function buildSendBuffer(data) {
  if (typeof data === 'number') {
    const buffer = new ArrayBuffer(1);
    new DataView(buffer).setUint8(0, data);
    return buffer;
  }
  if (Array.isArray(data)) {
    const buffer = new ArrayBuffer(data.length);
    const view = new DataView(buffer);
    data.forEach((b, i) => view.setUint8(i, b));
    return buffer;
  }
  // 字符串
  const buffer = new ArrayBuffer(data.length);
  const view = new DataView(buffer);
  for (let i = 0; i < data.length; i++) {
    view.setUint8(i, data.charCodeAt(i));
  }
  return buffer;
}

/** NanoEdge AI 异常告警标记字符串 */
const AI_ALERT_MARKER = '111';
/** AI 告警历史记录 storage key */
const AI_ALERT_HISTORY_KEY = 'ai_alert_history';

/**
 * 判断原始 BLE 数据是否为 NanoEdge AI 异常告警
 * @param {string} raw - 原始字符串（buf2Str 转换后）
 * @returns {boolean}
 */
function isAIAlertMessage(raw) {
  if (!raw || typeof raw !== 'string') return false;
  return raw.trim() === AI_ALERT_MARKER;
}

/** Storage key：灯光开关状态 */
const DEVICE_LIGHT_KEY  = 'device_light';
/** Storage key：空调开关状态 */
const DEVICE_AC_KEY     = 'device_ac';
/** Storage key：插座开关状态 */
const DEVICE_SOCKET_KEY = 'device_socket';
/** Storage key：门锁开关状态 */
const DEVICE_LOCK_KEY   = 'device_lock';
/** Storage key：当前场景模式 */
const DEVICE_SCENE_KEY  = 'scene_current_mode';
/** Storage key：当前系统模式 */
const DEVICE_SYS_MODE_KEY = 'system_mode';
/** Storage key：最新完整设备数据（JSON 字符串） */
const DEVICE_LATEST_DATA_KEY = 'device_latest_data';

// ============================================================
//  2 字节通讯协议命令定义
// ============================================================

/** 设备控制命令 */
const CMD = {
  // ---- 设备控制（A0~AF） ----
  LED_ON:       [0xA0, 0x01],
  LED_OFF:      [0xA0, 0x00],
  AC_ON:        [0xA1, 0x01],
  AC_OFF:       [0xA1, 0x00],
  SOCKET_ON:    [0xA2, 0x01],
  SOCKET_OFF:   [0xA2, 0x00],
  LOCK_ON:      [0xA3, 0x01],  // 门锁-锁定
  LOCK_OFF:     [0xA3, 0x00],  // 门锁-解锁

  // ---- 场景模式（B0） ----
  SCENE_SLEEP:  [0xB0, 0x01],
  SCENE_STUDY:  [0xB0, 0x02],
  SCENE_REST:   [0xB0, 0x03],

  // ---- 系统模式（B1） ----
  MODE_NORMAL:  [0xB1, 0x00],
  MODE_ECO:     [0xB1, 0x01],
  MODE_SAFE:    [0xB1, 0x02],

  // ---- 查询/同步（C0） ----
  QUERY_ALL:    [0xC0, 0x00],
  QUERY_SENSOR: [0xC0, 0x01],
};

// ---- 告警/模式/状态映射表（全局复用） ----
const ALARM_TEXT_MAP  = { 0: '正常', 1: '提醒', 2: '警告', 3: '紧急' };
const ALARM_CLASS_MAP = ['alarm-ok', 'alarm-info', 'alarm-warn', 'alarm-urgent'];
const MODE_TEXT_MAP   = { 0: '正常', 1: '睡眠', 2: '宿管' };
const HEALTH_TEXT_MAP = { 0: '⚠️ 异常', 1: '✅ 健康' };
const ALARM_EMOJI_MAP = { 0: '🟢', 1: '🟡', 2: '🟠', 3: '🔴' };

/**
 * 从 globalData + pageData 获取 BLE 上下文（统一 fallback 链）
 * @param {Object} pageData - 页面 this.data 对象
 * @returns {{ deviceId:string, serviceId:string, readCharId:string, writeCharId:string }}
 */
function getBLEContext(pageData) {
  const app = getApp();
  const ctx = app.globalData.bleContext || {};
  return {
    deviceId:    pageData.connectedDeviceId || ctx.deviceId    || '',
    serviceId:   pageData.serviceId          || ctx.serviceId   || '',
    readCharId:  pageData.readCharId         || ctx.readCharId  || '',
    writeCharId: pageData.writeCharId        || ctx.writeCharId || ''
  };
}

/**
 * 解析逗号分隔的传感器数据字符串（旧版兼容）
 * 格式: "温度,湿度,光照,人体"  (兼容中英文逗号)
 * @param {string} raw - 原始数据字符串
 * @returns {{temp:number, humidity:number, light:number, human:number}|null}
 */
function parseSensorData(raw) {
  if (!raw) return null;
  let parts = raw.replace(/，/g, ',').split(',');
  if (parts.length < 4) return null;
  const temp = parseFloat(parts[0]);
  const humidity = parseFloat(parts[1]);
  const light = parseFloat(parts[2]);
  const human = parseFloat(parts[3]);
  if (isNaN(temp) || isNaN(humidity) || isNaN(light) || isNaN(human)) return null;
  return { temp, humidity, light, human };
}

/**
 * 解析新版 JSON 设备数据
 * 格式: {"t":26.5,"v":220,"c":0.5,"p":110,"e":0.123,"s":0,"d":0,"h":0,"l":1,"k":0,"x":0,"z":1,"m":0,"a":0,"u":1}
 *
 * 字段映射:
 *   t=温度(℃)  v=电压(V)   c=电流(A)   p=功率(W)   e=累计电量(kWh)
 *   s=烟雾(0/1) d=门磁(0/1) h=人体(0/1)
 *   l=灯(0/1)  k=插座(0/1) x=空调(0/1)  z=锁(0/1)
 *   m=模式(0正常/1睡眠/2宿管)  a=告警级别(0-3)  u=传感器健康(0异常/1正常)
 *
 * @param {string} raw - 原始 JSON 字符串
 * @returns {DeviceData|null} 标准化设备数据对象，解析失败返回 null
 */
function parseDeviceData(raw) {
  if (!raw || typeof raw !== 'string') return null;

  // 尝试匹配 JSON 对象格式（兼容串口可能带的前后缀字符）
  const jsonMatch = raw.match(/\{[^}]*\}/);
  if (!jsonMatch) return null;

  let obj;
  try {
    obj = JSON.parse(jsonMatch[0]);
  } catch (e) {
    return null;
  }

  // 必须有温度字段才认为是有效数据
  if (obj.t === undefined || isNaN(parseFloat(obj.t))) return null;

  const now = formatNow();

  return {
    // 电能计量
    temp:    parseFloat(obj.t) || 0,      // 温度 ℃
    voltage: parseFloat(obj.v) || 0,      // 电压 V
    current: parseFloat(obj.c) || 0,      // 电流 A
    power:   parseFloat(obj.p) || 0,      // 功率 W
    energy:  parseFloat(obj.e) || 0,      // 累计电量 kWh

    // 安防传感 (0=正常/关闭, 1=报警/开启)
    smoke: parseInt(obj.s) || 0,          // 烟雾
    door:  parseInt(obj.d) || 0,          // 门磁
    human: parseInt(obj.h) || 0,          // 人体

    // 设备开关 (0=关, 1=开)
    light:  parseInt(obj.l) || 0,         // 灯
    socket: parseInt(obj.k) || 0,         // 插座
    ac:     parseInt(obj.x) || 0,         // 空调
    lock:   parseInt(obj.z) || 0,         // 锁

    // 系统状态
    mode:   parseInt(obj.m) || 0,         // 0=正常, 1=睡眠, 2=宿管
    alarm:  parseInt(obj.a) || 0,         // 0=正常, 1=提醒, 2=警告, 3=紧急
    health: parseInt(obj.u) || 0,         // 0=异常, 1=正常

    // 元数据
    timestamp: now,
    raw: raw
  };
}

/**
 * 返回当前时间 HH:MM:SS 格式
 */
function formatNow() {
  const d = new Date();
  const pad = n => n.toString().padStart(2, '0');
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

module.exports = {
  buf2Str, buf2Hex, getBLEParams, getBLEContext, buildSendBuffer,
  parseSensorData, parseDeviceData, formatNow, isAIAlertMessage,
  AI_ALERT_MARKER, AI_ALERT_HISTORY_KEY,
  DEVICE_LIGHT_KEY, DEVICE_AC_KEY, DEVICE_SOCKET_KEY, DEVICE_LOCK_KEY,
  DEVICE_SCENE_KEY, DEVICE_SYS_MODE_KEY, DEVICE_LATEST_DATA_KEY,
  ALARM_TEXT_MAP, ALARM_CLASS_MAP, MODE_TEXT_MAP,
  HEALTH_TEXT_MAP, ALARM_EMOJI_MAP,
  CMD
};
