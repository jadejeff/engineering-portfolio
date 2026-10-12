const app = getApp();
const {
  buf2Str, getBLEParams, getBLEContext, buildSendBuffer,
  parseDeviceData, formatNow, CMD, isAIAlertMessage, AI_ALERT_HISTORY_KEY,
  DEVICE_LIGHT_KEY, DEVICE_AC_KEY, DEVICE_SOCKET_KEY, DEVICE_LOCK_KEY,
  DEVICE_LATEST_DATA_KEY,
  ALARM_TEXT_MAP, ALARM_CLASS_MAP, MODE_TEXT_MAP, HEALTH_TEXT_MAP
} = require('../../utils/ble-utils');

Page({
  data: {
    // BLE 参数
    connectedDeviceId: "",
    serviceId: "",
    readCharId: "",
    writeCharId: "",

    // 电能计量
    temp: "--",
    voltage: "--",
    current: "--",
    power: "--",
    energy: "--",

    // 安防传感
    smoke: 0,
    door: 0,
    human: 0,

    // 设备状态
    light: 0,
    socket: 0,
    ac: 0,
    lock: 0,

    // 系统状态
    mode: 0,
    alarm: 0,
    health: 1,

    // UI 状态
    lastUpdate: "",
    bleDisconnected: false,
    dataLoaded: false,
    isRefreshing: false,
    isOffline: false,  // 离线快照模式

    // 功率圆环仪表盘
    powerPercent: 0,
    powerAngle: 0,

    // 模式文本
    modeText: "--",
    alarmText: "--",
    alarmClass: "",
    healthText: "--",

    // AI 异常告警横幅
    showAIBanner: false,
    aiBannerClosing: false,

    // AI 告警历史记录
    aiAlertHistory: []
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'readCharId', 'writeCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      readCharId: params.readCharId,
      writeCharId: params.writeCharId
    });

    this.setData({ lastUpdate: formatNow() });

    // 从 storage 恢复上次设备状态
    this.syncStorageStates();

    // 加载 AI 告警历史
    this.loadAIAlertHistory();

    // 监听 BLE 数据
    this.listenSensorData();

    // 注册 AI 异常告警回调
    this._aiAlertCb = this.handleAIAlert.bind(this);
    app.onAIAlert(this._aiAlertCb);

    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ monitor 页收到断连通知:', deviceId);
      this.setData({ bleDisconnected: true, isOffline: true });
      // 从 globalData 恢复最后一次数据快照
      const latest = app.globalData.latestDeviceData;
      if (latest) {
        this.applyDeviceData(latest);
      }
      if (this._isVisible !== false) {
        setTimeout(() => {
          wx.showModal({
            title: "设备已离线",
            content: "蓝牙连接已断开。当前显示最后一次数据快照。",
            showCancel: false,
            confirmText: "知道了",
            success: () => { wx.navigateBack({ delta: 2 }); }
          });
        }, 2000);
      }
    };
    app.onBLEDisconnect(this._disconnectCb);
  },

  onShow() {
    this._isVisible = true;
    this.syncStorageStates();
    // 尝试从 globalData 恢复最新数据
    const latest = app.globalData.latestDeviceData;
    if (latest && !this.data.dataLoaded) {
      this.applyDeviceData(latest);
    }
  },

  onHide() {
    this._isVisible = false;
  },

  onUnload() {
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
    if (this._aiAlertCb) {
      app.offAIAlert(this._aiAlertCb);
      this._aiAlertCb = null;
    }
    // 清除横幅自动关闭定时器
    if (this._bannerTimer) {
      clearTimeout(this._bannerTimer);
      this._bannerTimer = null;
    }
  },

  // ============================================================
  //  从 storage 恢复设备状态
  // ============================================================
  syncStorageStates() {
    try {
      const lightOn  = wx.getStorageSync(DEVICE_LIGHT_KEY);
      const acOn     = wx.getStorageSync(DEVICE_AC_KEY);
      const socketOn = wx.getStorageSync(DEVICE_SOCKET_KEY);
      const lockOn   = wx.getStorageSync(DEVICE_LOCK_KEY);

      // 只有当数据还没从 BLE 加载时才用 storage 状态
      if (!this.data.dataLoaded) {
        this.setData({
          light:  lightOn === true ? 1 : 0,
          ac:     acOn === true ? 1 : 0,
          socket: socketOn === true ? 1 : 0,
          lock:   lockOn === true ? 1 : 0,
        });
      }
    } catch (e) { /* ignore */ }
  },

  // ============================================================
  //  刷新按钮 — 请求全量状态 (C0 00)
  // ============================================================
  refreshStatus() {
    if (this.data.isRefreshing) return;
    if (this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开", icon: "none" });
      return;
    }

    const { connectedDeviceId, serviceId, writeCharId } = this.data;
    if (!connectedDeviceId || !writeCharId) {
      wx.showToast({ title: "蓝牙未连接", icon: "none" });
      return;
    }

    this.setData({ isRefreshing: true });

    const buffer = buildSendBuffer(CMD.QUERY_ALL);

    console.log('🔄 [monitor] 发送刷新请求 C0 00');

    wx.writeBLECharacteristicValue({
      deviceId: connectedDeviceId,
      serviceId: serviceId,
      characteristicId: writeCharId,
      value: buffer,
      success: () => {
        console.log('✅ [monitor] 刷新请求已发送');
        this.setData({ isRefreshing: false });
        wx.showToast({ title: "已刷新", icon: "success", duration: 1000 });
      },
      fail: (err) => {
        console.error('❌ [monitor] 刷新请求失败:', JSON.stringify(err));
        this.setData({ isRefreshing: false });
        wx.showToast({ title: "刷新失败", icon: "error" });
      }
    });
  },

  // ============================================================
  //  监听 BLE 数据
  // ============================================================
  listenSensorData() {
    const ctx = app.globalData.bleContext || {};
    const deviceId = this.data.connectedDeviceId || ctx.deviceId;
    const serviceId = this.data.serviceId || ctx.serviceId;
    const readCharId = this.data.readCharId || ctx.readCharId;

    if (!deviceId || !serviceId || !readCharId) {
      console.warn('⚠️ monitor 缺少蓝牙参数');
      return;
    }

    // notify 已在 index.js 中统一开启，此处不再重复调用

    // 通过中央分发器订阅数据（替代直接注册 wx.onBLECharacteristicValueChange）
    this._bleDataCb = (data) => {
      wx.setStorageSync(DEVICE_LATEST_DATA_KEY, JSON.stringify(data));
      this.applyDeviceData(data);
    };
    app.onBLEData(this._bleDataCb);

    // Fallback：如果中央分发器未启动，尝试直接监听（兼容旧连接流程）
    if (!app._bleDataListenerReady) {
      console.warn('⚠️ 中央分发器未启动，启用备用监听');
      this._charChangeCb = (res) => {
        if (!res.value) return;
        let raw = buf2Str(res.value);
        if (!raw) return;

        // NanoEdge AI 异常告警检测（fallback 路径也需判断）
        if (isAIAlertMessage(raw)) {
          console.warn('⚠️ [fallback] NanoEdge AI 异常告警: "111"');
          this.handleAIAlert();
          return;
        }

        const data = parseDeviceData(raw);
        if (!data) return;
        app.globalData.latestDeviceData = data;
        this.applyDeviceData(data);
      };
      wx.onBLECharacteristicValueChange(this._charChangeCb);
    }
  },

  // ============================================================
  //  将 DeviceData 应用到页面
  // ============================================================
  applyDeviceData(data) {
    // 功率圆环仪表盘：额定功率设为 3000W，计算百分比
    const ratedPower = 100;
    const powerPercent = Math.min(100, Math.max(0, (data.power / ratedPower) * 100));
    const powerAngle = (powerPercent / 100) * 360;

    // 模式文本
    const modeText = MODE_TEXT_MAP[data.mode] || '--';

    // 告警级别
    const alarmText = ALARM_TEXT_MAP[data.alarm] || '--';
    const alarmClass = ALARM_CLASS_MAP[data.alarm] || '';

    // 传感器健康
    const healthText = HEALTH_TEXT_MAP[data.health] || '--';

    this.setData({
      temp:     data.temp.toFixed(1),
      voltage:  data.voltage.toFixed(1),
      current:  data.current.toFixed(3),
      power:    data.power.toFixed(1),
      energy:   data.energy.toFixed(4),

      smoke: data.smoke,
      door:  data.door,
      human: data.human,

      light:  data.light,
      socket: data.socket,
      ac:     data.ac,
      lock:   data.lock,

      mode:  data.mode,
      alarm: data.alarm,
      health: data.health,

      lastUpdate: formatNow(),
      dataLoaded: true,

      powerPercent: powerPercent,
      powerAngle: powerAngle,

      modeText, alarmText, alarmClass, healthText
    });

    // 同步设备状态到 storage
    wx.setStorageSync(DEVICE_LIGHT_KEY, !!data.light);
    wx.setStorageSync(DEVICE_AC_KEY, !!data.ac);
    wx.setStorageSync(DEVICE_SOCKET_KEY, !!data.socket);
    wx.setStorageSync(DEVICE_LOCK_KEY, !!data.lock);
  },

  // ============================================================
  //  NanoEdge AI 异常告警处理
  // ============================================================

  /**
   * 处理 AI 异常告警（由 app.js 中央分发器或 fallback 路径调用）
   */
  handleAIAlert() {
    const now = new Date();
    const pad = n => n.toString().padStart(2, '0');
    const timeStr = `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}`;

    const alertItem = {
      time: timeStr,
      timestamp: now.getTime(),
      message: '⚠️ AI 异常检测告警：设备运行状态异常！'
    };

    // 追加到历史记录，最多保留 50 条
    const history = [alertItem].concat(this.data.aiAlertHistory).slice(0, 50);
    this.setData({ aiAlertHistory: history });
    // 持久化存储
    wx.setStorageSync(AI_ALERT_HISTORY_KEY, history);

    // 显示告警横幅（先清除旧定时器）
    if (this._bannerTimer) {
      clearTimeout(this._bannerTimer);
      this._bannerTimer = null;
    }
    this.setData({ showAIBanner: true, aiBannerClosing: false });

    // 手机振动提醒
    try {
      wx.vibrateLong({
        success: () => console.log('📳 AI 告警振动已触发'),
        fail: (err) => console.warn('振动调用失败:', err)
      });
    } catch (e) {
      console.warn('振动 API 不可用:', e);
    }

    // 显示弹窗提示
    wx.showModal({
      title: '⚠️ AI 异常告警',
      content: 'NanoEdge AI 检测到设备运行状态异常！',
      confirmText: '知道了',
      showCancel: false
    });

    // 5 秒后自动隐藏横幅
    this._bannerTimer = setTimeout(() => {
      this.dismissAIBanner();
    }, 5000);
  },

  /**
   * 手动关闭告警横幅
   */
  dismissAIBanner() {
    if (this._bannerTimer) {
      clearTimeout(this._bannerTimer);
      this._bannerTimer = null;
    }
    this.setData({ aiBannerClosing: true });
    // 等动画结束后移除
    setTimeout(() => {
      this.setData({ showAIBanner: false, aiBannerClosing: false });
    }, 300);
  },

  /**
   * 从 storage 加载告警历史记录
   */
  loadAIAlertHistory() {
    try {
      const history = wx.getStorageSync(AI_ALERT_HISTORY_KEY) || [];
      if (history.length > 0) {
        this.setData({ aiAlertHistory: history });
      }
    } catch (e) {
      console.warn('加载 AI 告警历史失败:', e);
    }
  }
});
