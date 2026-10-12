const app = getApp();
const {
  buf2Str, getBLEParams, parseSensorData, parseDeviceData, formatNow
} = require('../../utils/ble-utils');
const { TEMPLATES, sendAlarmWithCooldown } = require('../../utils/subscribe-utils');

const STORAGE_KEY = 'dorm_history';
const MAX_HISTORY = 500;

Page({
  data: {
    connectedDeviceId: "",
    serviceId: "",
    readCharId: "",

    // 环境数据
    temp: "--",
    humidity: "--",   // 旧版兼容

    // 电能计量（新）
    voltage: "--",
    current: "--",
    power: "--",
    energy: "--",

    // 安防传感（新）
    smoke: 0,
    door: 0,
    human: "--",

    // 设备状态（新）
    light: 0,
    socket: 0,
    ac: 0,
    lock: 0,

    // 系统状态
    mode: 0,
    alarm: 0,
    health: 1,

    tempBarWidth: 0,
    powerBarWidth: 0,     // 新格式：功率百分比进度条
    lightBarWidth: 0,     // 旧格式：光照亮度百分比进度条

    comfortText: "--",
    suggestionText: "--",

    lastUpdate: "",
    bleDisconnected: false,
    dataLoaded: false,
    isNewFormat: false,  // 是否使用新 JSON 格式

    // 今日统计
    todayMaxTemp: "--",
    todayMinTemp: "--",
    todayAvgTemp: "--",
    todayAvgHum: "--",
    todayAvgPower: "--",      // 今日平均功率 W
    todayMaxPower: "--",      // 今日最大功率 W
    todayTotalEnergy: "--",   // 日累计用电量 kWh
    todayHumanCount: "0",
    todayDataCount: 0,

    // 顶部告警条
    showAlert: false,
    alertMessage: "",
    alertType: "",

    // 卡片告警红框
    tempAlarm: false,
    smokeAlarm: false,
    doorAlarm: false,
    humanAlarm: false,
    powerAlarm: false,

    // 事件日志
    showEventLog: false,
    events: [],

    // 订阅消息
    subscribed: false
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'readCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      readCharId: params.readCharId
    });

    this.setData({ lastUpdate: formatNow() });

    // 初始化告警状态追踪
    this._alarmState = {
      tempHigh: false, tempLow: false, humHigh: false,
      smokeAlert: false, doorOpen: false,
      humanNight: false, powerHigh: false
    };

    // 接入蓝牙实时数据
    this.listenSensorData();

    // 加载历史
    this.loadHistory();
    this.setData({ ...this.calcTodayStats() });

    // 订阅状态
    this.setData({ subscribed: !!wx.getStorageSync('alarm_subscribed') });

    // 注册全局断连回调
    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ dorm 页收到断连通知:', deviceId);
      this.setData({ bleDisconnected: true });
      if (this._isVisible !== false) {
        wx.showModal({
          title: "蓝牙已断开",
          content: "设备连接已断开，监测已中断",
          showCancel: false,
          success: () => { wx.navigateBack({ delta: 2 }); }
        });
      }
    };
    app.onBLEDisconnect(this._disconnectCb);

    // 注册全局告警回调
    this._alertCb = (alert) => {
      if (this._isVisible === false) {
        wx.showModal({
          title: '⚠️ 寝室告警',
          content: alert.message,
          showCancel: false,
          confirmText: '知道了'
        });
      }
    };
    app.onAlert(this._alertCb);
  },

  onShow() {
    this._isVisible = true;
    // 切回页面时刷新今日统计（离开期间可能有新数据写入 Storage）
    const stats = this.calcTodayStats();
    if (stats.todayDataCount !== this.data.todayDataCount) {
      this.setData(stats);
      this.loadHistory();
    }
  },
  onHide() { this._isVisible = false; },

  onUnload() {
    if (this._alertTimer) { clearTimeout(this._alertTimer); this._alertTimer = null; }
    if (this._bleDataCb) {
      app.offBLEData(this._bleDataCb);
      this._bleDataCb = null;
    }
    if (this._charChangeCb) {
      wx.offBLECharacteristicValueChange(this._charChangeCb);
      this._charChangeCb = null;
    }
    if (this._disconnectCb) {
      app.offBLEDisconnect(this._disconnectCb);
      this._disconnectCb = null;
    }
    if (this._alertCb) {
      app.offAlert(this._alertCb);
      this._alertCb = null;
    }
  },

  // ============================================================
  //  监听单片机实时数据（新旧格式兼容）
  // ============================================================
  listenSensorData() {
    const ctx = app.globalData.bleContext || {};
    const deviceId = this.data.connectedDeviceId || ctx.deviceId;
    const serviceId = this.data.serviceId || ctx.serviceId;
    const readCharId = this.data.readCharId || ctx.readCharId;

    if (!deviceId || !serviceId || !readCharId) {
      console.warn('⚠️ dorm 缺少蓝牙参数');
      return;
    }

    // notify 已在 index.js 中统一开启，此处不再重复调用
    // wx.notifyBLECharacteristicValueChange 调用已移至中央监听器

    // 通过中央分发器订阅数据
    this._bleDataCb = (data, raw) => {
      console.log('📥 dorm收到:', raw);
      if (data) {
        this._processNewFormatData(data);
      } else {
        // 回退旧格式
        const oldData = parseSensorData(raw);
        if (oldData) this._processOldFormatData(oldData);
      }
    };
    app.onBLEData(this._bleDataCb);

    // Fallback：如果中央分发器未启动
    if (!app._bleDataListenerReady) {
      console.warn('⚠️ 中央分发器未启动，启用备用监听');
      this._charChangeCb = (res) => {
        if (!res.value) return;
        let raw = buf2Str(res.value);
        if (!raw) return;
        let deviceData = parseDeviceData(raw);
        if (deviceData) {
          this._processNewFormatData(deviceData);
        } else {
          const oldData = parseSensorData(raw);
          if (oldData) this._processOldFormatData(oldData);
        }
      };
      wx.onBLECharacteristicValueChange(this._charChangeCb);
    }
  },

  // ============================================================
  //  新版 JSON 数据处理
  // ============================================================
  _processNewFormatData(data) {
    let humanText = data.human > 0 ? '有人' : '无人';

    // 进度条
    let tempBarWidth = Math.min(100, Math.max(0, ((data.temp - 10) / 25) * 100));
    let powerBarWidth = Math.min(100, Math.max(0, (data.power / 3000) * 100));

    // 舒适度评估
    let issues = [];
    if (data.temp > 35) issues.push({ comfort: '偏热', suggestion: '开窗通风降温' });
    if (data.temp < 10) issues.push({ comfort: '偏冷', suggestion: '关窗注意保暖' });
    if (data.smoke) issues.push({ comfort: '烟雾报警', suggestion: '立即检查火源!' });
    if (data.door) issues.push({ comfort: '门未关', suggestion: '请检查门是否关好' });
    if (data.power > 2000) issues.push({ comfort: '高功率', suggestion: '注意用电安全' });
    let comfortText = issues.length ? issues.map(i => i.comfort).join('/') : '舒适';
    let suggestionText = issues.length ? issues.map(i => i.suggestion).join('；') : '环境良好';

    // 存入历史
    this.saveToHistory(data, true);

    // 统计 + 告警
    const stats = this.calcTodayStats();
    const alarms = this.evalAlertsNew(data);

    this.setData({
      temp: data.temp.toFixed(1),
      voltage: data.voltage.toFixed(1),
      current: data.current.toFixed(3),
      power: data.power.toFixed(1),
      energy: data.energy.toFixed(4),
      smoke: data.smoke,
      door: data.door,
      human: humanText,
      light: data.light,
      socket: data.socket,
      ac: data.ac,
      lock: data.lock,
      mode: data.mode,
      alarm: data.alarm,
      health: data.health,
      tempBarWidth, powerBarWidth,
      comfortText, suggestionText,
      lastUpdate: formatNow(),
      dataLoaded: true,
      isNewFormat: true,
      ...stats,
      ...alarms
    });
  },

  // ============================================================
  //  旧版逗号格式数据处理（过渡兼容）
  // ============================================================
  _processOldFormatData(data) {
    let human = data.human > 0 ? '有人' : '无人';
    let tempBarWidth = Math.min(100, Math.max(0, ((data.temp - 10) / 25) * 100));
    let lightBarWidth = Math.min(100, Math.max(0, (data.light / 500) * 100));

    let issues = [];
    if (data.temp > 35) issues.push({ comfort: '偏热', suggestion: '开窗通风降温' });
    if (data.temp < 10) issues.push({ comfort: '偏冷', suggestion: '关窗注意保暖' });
    if (data.humidity > 80) issues.push({ comfort: '潮湿', suggestion: '除湿通风' });
    if (data.light < 100) issues.push({ comfort: '偏暗', suggestion: '适当开灯' });
    let comfortText = issues.length ? issues.map(i => i.comfort).join('/') : '舒适';
    let suggestionText = issues.length ? issues.map(i => i.suggestion).join('；') : '环境良好';

    // 转换为新格式存储
    const newData = {
      temp: data.temp, voltage: 0, current: 0, power: 0, energy: 0,
      smoke: 0, door: 0, human: data.human,
      light: 0, socket: 0, ac: 0, lock: 0,
      mode: 0, alarm: 0, health: 1,
      timestamp: formatNow(), raw: ''
    };
    this.saveToHistory(newData, false);

    const stats = this.calcTodayStats();
    const alarms = this.evalAlertsOld(data);

    this.setData({
      temp: data.temp.toFixed(1),
      humidity: data.humidity.toFixed(0),
      human: human,
      tempBarWidth, lightBarWidth,
      comfortText, suggestionText,
      lastUpdate: formatNow(),
      dataLoaded: true,
      isNewFormat: false,
      ...stats,
      ...alarms
    });
  },

  // ============================================================
  //  本地存储
  // ============================================================
  saveToHistory(data, isNew) {
    try {
      const entry = {
        time: formatNow(),
        ts: Date.now(),
        isNew: isNew,
        temp: data.temp,
        humidity: data.humidity || 0,
        light: data.power || data.light || 0,  // 新格式存功率
        human: data.human || 0,
        power: data.power || 0,
        energy: data.energy || 0,
        voltage: data.voltage || 0,
        current: data.current || 0,
        smoke: data.smoke || 0,
        door: data.door || 0,
        alarm: data.alarm || 0
      };
      let history = wx.getStorageSync(STORAGE_KEY) || [];
      history.push(entry);
      if (history.length > MAX_HISTORY) history = history.slice(-MAX_HISTORY);
      wx.setStorageSync(STORAGE_KEY, history);
    } catch (e) { console.warn('saveToHistory 失败:', e); }
  },

  loadHistory() {
    try {
      const history = wx.getStorageSync(STORAGE_KEY) || [];
      const events = [];
      history.forEach(entry => {
        const h = parseInt(entry.time.split(':')[0]);
        const isNight = h >= 23 || h < 6;
        if (entry.temp > 35) events.push({ time: entry.time, type: 'temp_high', icon: '🌡️', msg: `温度升至 ${entry.temp.toFixed(1)}℃` });
        if (entry.temp < 10) events.push({ time: entry.time, type: 'temp_low', icon: '🌡️', msg: `温度降至 ${entry.temp.toFixed(1)}℃` });
        if (entry.humidity > 80) events.push({ time: entry.time, type: 'hum_high', icon: '💧', msg: `湿度升至 ${entry.humidity.toFixed(0)}%` });
        if (entry.smoke > 0) events.push({ time: entry.time, type: 'smoke', icon: '🔥', msg: '烟雾传感器报警!' });
        if (entry.door > 0) events.push({ time: entry.time, type: 'door', icon: '🚪', msg: '门磁检测到门未关闭' });
        if (entry.power > 2000) events.push({ time: entry.time, type: 'power_high', icon: '⚡', msg: `功率过高 ${entry.power.toFixed(0)}W` });
        if (entry.human > 0 && isNight) events.push({ time: entry.time, type: 'human_night', icon: '🚶', msg: '深夜检测到人员活动' });
      });
      this.setData({ events: events.slice(-30) });
    } catch (e) { console.warn('loadHistory 失败:', e); }
  },

  // ============================================================
  //  今日统计
  // ============================================================
  calcTodayStats() {
    try {
      const history = wx.getStorageSync(STORAGE_KEY) || [];
      const today = new Date().toDateString();
      const todayData = history.filter(e => new Date(e.ts).toDateString() === today);

      if (todayData.length === 0) return {
        todayMaxTemp: '--', todayMinTemp: '--', todayAvgTemp: '--',
        todayAvgHum: '--', todayAvgPower: '--', todayMaxPower: '--',
        todayTotalEnergy: '--',
        todayHumanCount: '0', todayDataCount: 0
      };

      const temps = todayData.map(e => e.temp);
      const hums = todayData.map(e => e.humidity).filter(v => v > 0);
      const powers = todayData.map(e => e.power).filter(v => v > 0);
      const humanHits = todayData.filter(e => e.human > 0).length;

      const result = {
        todayMaxTemp: Math.max(...temps).toFixed(1),
        todayMinTemp: Math.min(...temps).toFixed(1),
        todayAvgTemp: (temps.reduce((a, b) => a + b, 0) / temps.length).toFixed(1),
        todayHumanCount: humanHits.toString(),
        todayDataCount: todayData.length
      };

      if (hums.length > 0) {
        result.todayAvgHum = Math.round(hums.reduce((a, b) => a + b, 0) / hums.length).toString();
      } else {
        result.todayAvgHum = '--';
      }

      if (powers.length > 0) {
        // 最大功率 + 平均功率
        result.todayMaxPower = Math.max(...powers).toFixed(0);
        const avgPower = powers.reduce((a, b) => a + b, 0) / powers.length;
        result.todayAvgPower = avgPower.toFixed(0);

        // 日累计用电量估算: Σ(功率 × 采样间隔秒 / 3600) → kWh
        let totalEnergyWh = 0;
        for (let i = 1; i < todayData.length; i++) {
          const dt = (todayData[i].ts - todayData[i - 1].ts) / 1000; // 采样间隔秒
          if (dt > 0 && dt < 3600) { // 剔除异常长间隔
            totalEnergyWh += (todayData[i].power || 0) * dt / 3600;
          }
        }
        result.todayTotalEnergy = (totalEnergyWh / 1000).toFixed(4); // Wh → kWh
      } else {
        result.todayMaxPower = '--';
        result.todayAvgPower = '--';
        result.todayTotalEnergy = '--';
      }

      return result;
    } catch (e) {
      console.warn('calcTodayStats 失败:', e);
      return {};
    }
  },

  // ============================================================
  //  新版告警判定（15 字段完整逻辑）
  // ============================================================
  evalAlertsNew(data) {
    const now = new Date();
    const hour = now.getHours();
    const isNight = hour >= 23 || hour < 6;

    const result = {
      tempAlarm: false, smokeAlarm: false,
      doorAlarm: false, humanAlarm: false, powerAlarm: false
    };
    let alertMsg = null, alertType = null;

    // ===== 检测顺序即优先级：烟雾 > 人体 > 门磁 > 功率 > 温度偏高 > 温度偏低 =====

    // -------- 1. 烟雾报警（最高优先级——生命安全）--------
    if (data.smoke > 0) {
      result.smokeAlarm = true;
      if (!this._alarmState.smokeAlert) {
        this._alarmState.smokeAlert = true;
        alertMsg = '🔥 烟雾传感器报警！请立即检查！';
        alertType = 'smoke';
        this.addEventLog({ time: formatNow(), type: 'smoke', icon: '🔥', msg: '烟雾传感器报警!' });
      }
    } else {
      this._alarmState.smokeAlert = false;
    }

    // -------- 2. 深夜有人（安全）--------
    if (data.human > 0 && isNight) {
      result.humanAlarm = true;
      if (!this._alarmState.humanNight) {
        this._alarmState.humanNight = true;
        alertMsg = alertMsg || '🚨 深夜检测到人员活动！';
        alertType = alertType || 'human';
        this.addEventLog({ time: formatNow(), type: 'human_night', icon: '🚶', msg: '深夜检测到人员活动' });
      }
    } else if (data.human === 0) {
      this._alarmState.humanNight = false;
    }

    // -------- 3. 深夜门未关（安全）--------
    if (data.door > 0 && isNight) {
      result.doorAlarm = true;
      if (!this._alarmState.doorOpen) {
        this._alarmState.doorOpen = true;
        alertMsg = alertMsg || '🚪 深夜门未关闭！';
        alertType = alertType || 'door';
        this.addEventLog({ time: formatNow(), type: 'door_night', icon: '🚪', msg: '深夜门未关闭' });
      }
    } else if (data.door === 0) {
      this._alarmState.doorOpen = false;
    }

    // -------- 4. 高功率警告（用电安全）--------
    if (data.power > 2500) {
      result.powerAlarm = true;
      if (!this._alarmState.powerHigh) {
        this._alarmState.powerHigh = true;
        alertMsg = alertMsg || `⚡ 功率过高 ${data.power.toFixed(0)}W！注意用电安全`;
        alertType = alertType || 'power';
        this.addEventLog({ time: formatNow(), type: 'power_high', icon: '⚡', msg: `功率过高 ${data.power.toFixed(0)}W` });
      }
    } else {
      this._alarmState.powerHigh = false;
    }

    // -------- 5. 温度偏高（舒适度）--------
    if (data.temp > 35) {
      result.tempAlarm = true;
      if (!this._alarmState.tempHigh) {
        this._alarmState.tempHigh = true;
        alertMsg = alertMsg || `⚠️ 温度偏高 ${data.temp.toFixed(1)}℃`;
        alertType = alertType || 'temp';
        this.addEventLog({ time: formatNow(), type: 'temp_high', icon: '🌡️', msg: `温度升至 ${data.temp.toFixed(1)}℃` });
      }
    } else {
      this._alarmState.tempHigh = false;
    }

    // -------- 6. 温度偏低（舒适度）--------
    if (data.temp < 10) {
      result.tempAlarm = true;
      if (!this._alarmState.tempLow) {
        this._alarmState.tempLow = true;
        alertMsg = alertMsg || `❄️ 温度偏低 ${data.temp.toFixed(1)}℃`;
        alertType = alertType || 'temp';
        this.addEventLog({ time: formatNow(), type: 'temp_low', icon: '🌡️', msg: `温度降至 ${data.temp.toFixed(1)}℃` });
      }
    } else {
      this._alarmState.tempLow = false;
    }

    // -------- 触发告警条 --------
    if (alertMsg) {
      result.showAlert = true;
      result.alertMessage = alertMsg;
      result.alertType = alertType;

      wx.vibrateShort({ type: 'medium' });
      app.addPendingAlert({ message: alertMsg, type: alertType, time: formatNow() });
      this.sendAlarmNotification(alertMsg);

      if (this._alertTimer) clearTimeout(this._alertTimer);
      this._alertTimer = setTimeout(() => {
        this.setData({ showAlert: false });
      }, 5000);
    }

    return result;
  },

  // ============================================================
  //  旧版告警（保留兼容）
  // ============================================================
  evalAlertsOld(data) {
    const now = new Date();
    const hour = now.getHours();
    const isNight = hour >= 23 || hour < 6;

    const result = { tempAlarm: false, humAlarm: false, lightAlarm: false, humanAlarm: false };
    let alertMsg = null, alertType = null;

    if (data.temp > 35) {
      result.tempAlarm = true;
      if (!this._alarmState.tempHigh) {
        this._alarmState.tempHigh = true;
        alertMsg = `⚠️ 温度偏高 ${data.temp.toFixed(1)}℃`;
        alertType = 'temp';
        this.addEventLog({ time: formatNow(), type: 'temp_high', icon: '🌡️', msg: `温度升至 ${data.temp.toFixed(1)}℃` });
      }
    } else { this._alarmState.tempHigh = false; }

    if (data.temp < 10) {
      result.tempAlarm = true;
      if (!this._alarmState.tempLow) {
        this._alarmState.tempLow = true;
        alertMsg = alertMsg || `❄️ 温度偏低 ${data.temp.toFixed(1)}℃`;
        alertType = alertType || 'temp';
        this.addEventLog({ time: formatNow(), type: 'temp_low', icon: '🌡️', msg: `温度降至 ${data.temp.toFixed(1)}℃` });
      }
    } else { this._alarmState.tempLow = false; }

    if (data.humidity > 80) {
      result.humAlarm = true;
      if (!this._alarmState.humHigh) {
        this._alarmState.humHigh = true;
        alertMsg = alertMsg || `💧 湿度过高 ${data.humidity.toFixed(0)}%`;
        alertType = alertType || 'hum';
        this.addEventLog({ time: formatNow(), type: 'hum_high', icon: '💧', msg: `湿度升至 ${data.humidity.toFixed(0)}%` });
      }
    } else { this._alarmState.humHigh = false; }

    if (data.light < 100 && !isNight) { result.lightAlarm = true; }

    if (data.human > 0 && isNight) {
      result.humanAlarm = true;
      if (!this._alarmState.humanNight) {
        this._alarmState.humanNight = true;
        alertMsg = alertMsg || '🚨 深夜检测到人员活动！';
        alertType = alertType || 'human';
        this.addEventLog({ time: formatNow(), type: 'human_night', icon: '🚶', msg: '深夜检测到人员活动' });
      }
    } else { this._alarmState.humanNight = false; }

    if (alertMsg) {
      result.showAlert = true;
      result.alertMessage = alertMsg;
      result.alertType = alertType;
      wx.vibrateShort({ type: 'medium' });
      app.addPendingAlert({ message: alertMsg, type: alertType, time: formatNow() });
      this.sendAlarmNotification(alertMsg);
      if (this._alertTimer) clearTimeout(this._alertTimer);
      this._alertTimer = setTimeout(() => { this.setData({ showAlert: false }); }, 5000);
    }

    return result;
  },

  // ============================================================
  //  事件日志
  // ============================================================
  addEventLog(event) {
    const events = [event, ...this.data.events].slice(0, 50);
    this.setData({ events });
  },

  toggleEventLog() {
    this.setData({ showEventLog: !this.data.showEventLog });
  },

  dismissAlert() {
    if (this._alertTimer) { clearTimeout(this._alertTimer); this._alertTimer = null; }
    this.setData({ showAlert: false });
  },

  // ============================================================
  //  订阅消息
  // ============================================================
  subscribeAlarm() {
    console.log('🔔 开始请求订阅消息...');
    wx.requestSubscribeMessage({
      tmplIds: [TEMPLATES.ALARM],
      success: (res) => {
        const status = res[TEMPLATES.ALARM];
        if (status === 'accept') {
          wx.setStorageSync('alarm_subscribed', true);
          this.setData({ subscribed: true });
          wx.showToast({ title: '订阅成功！有异常时会通知你', icon: 'success' });
        } else if (status === 'reject') {
          wx.showToast({ title: '你拒绝了订阅，异常时不会通知', icon: 'none' });
        } else {
          wx.showToast({ title: '订阅结果: ' + status, icon: 'none' });
        }
      },
      fail: (err) => {
        console.error('❌ 订阅消息失败:', JSON.stringify(err));
        wx.showModal({
          title: '订阅失败',
          content: '错误码: ' + (err.errCode || '未知') + '\n错误信息: ' + (err.errMsg || '未知'),
          showCancel: false
        });
      }
    });
  },

  sendAlarmNotification(alertMsg) {
    if (!TEMPLATES.ALARM) {
      console.log('📩 模板ID未配置，跳过订阅消息发送');
      return;
    }
    const templateData = {
      thing1: { value: alertMsg.length > 20 ? alertMsg.substring(0, 17) + '...' : alertMsg },
      thing2: { value: '寝室监测设备' },
      time3: { value: new Date().toLocaleString('zh-CN', { hour12: false }) }
    };
    sendAlarmWithCooldown(TEMPLATES.ALARM, templateData, 'pages/dorm/dorm')
      .then((res) => {
        if (res.skipped) return;
        if (res.errCode === 43101) {
          wx.removeStorageSync('alarm_subscribed');
          this.setData({ subscribed: false });
        }
      })
      .catch((err) => { console.error('📩 订阅消息云函数异常:', err); });
  },

  // ============================================================
  //  断开连接
  // ============================================================
  disconnect() {
    wx.showModal({
      title: "断开连接",
      content: "确定要断开蓝牙连接吗？",
      success: (res) => {
        if (!res.confirm) return;
        this.doDisconnect();
      }
    });
  },

  doDisconnect() {
    wx.closeBLEConnection({
      deviceId: this.data.connectedDeviceId,
      success: () => {
        app.globalData.bleContext = null;
        app.globalData.bleConnected = false;
        wx.removeStorageSync('bleContext');
        wx.showToast({ title: "已断开连接" });
        setTimeout(() => { wx.navigateBack({ delta: 2 }); }, 500);
      },
      fail: (err) => {
        console.error('❌ 断开失败:', err);
        wx.showToast({ title: "断开失败", icon: "error" });
      }
    });
  }
});
