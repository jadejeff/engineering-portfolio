App({
  onLaunch() {
    // 初始化云开发
    if (wx.cloud) {
      wx.cloud.init({
        env: 'badminton-d8gt3u4vsb4e8ae2c',
        traceUser: true
      });
    }

    // 读取蓝牙上下文（上次连接的设备信息）
    const bleCtx = wx.getStorageSync('bleContext') || null;
    this.globalData.bleContext = bleCtx;

    // 初始化蓝牙断连监听的回调集合
    this._bleDisconnectCallbacks = [];

    // ✅ 标记是否已提示过重连，避免每次切前台都弹窗
    this._hasPromptedReconnect = false;

    // 全局告警回调集合 + 待处理告警列表
    this._alertCallbacks = [];
    this.globalData.pendingAlerts = wx.getStorageSync('pending_alerts') || [];

    // BLE 中央数据分发器回调集合
    this._bleDataCallbacks = [];
    // NanoEdge AI 异常告警回调集合
    this._aiAlertCallbacks = [];
  },

  /**
   * 前台恢复时检测蓝牙连接状态，提示用户重连
   */
  onShow() {
    const { bleContext, bleConnected } = this.globalData;

    // ---- 断连重连检测 ----
    if (bleContext && !bleConnected && !this._hasPromptedReconnect) {
      setTimeout(() => {
        if (!this.globalData.bleConnected) {
          this._hasPromptedReconnect = true;
          wx.showModal({
            title: "蓝牙已断开",
            content: `检测到设备 "${bleCtx.deviceId}" 连接已断开，是否重新连接？`,
            confirmText: "去重连",
            cancelText: "稍后再说",
            success: (res) => {
              if (res.confirm) {
                this._hasPromptedReconnect = false;
                const pages = getCurrentPages();
                const currentPage = pages[pages.length - 1];
                if (currentPage.route !== 'pages/index/index') {
                  const idx = pages.findIndex(p => p.route === 'pages/index/index');
                  if (idx >= 0) {
                    wx.navigateBack({ delta: pages.length - 1 - idx });
                  } else {
                    wx.reLaunch({ url: '/pages/index/index' });
                  }
                }
              }
            }
          });
        }
      }, 1500);
    }

    if (bleConnected) {
      this._hasPromptedReconnect = false;
    }

    // ---- 全局告警弹窗：切回前台时显示未读告警 ----
    setTimeout(() => {
      this.showPendingAlerts();
    }, 800);
  },

  onError(err) {
    console.error('[App Error]', err);
  },

  // ========== BLE 中央数据监听与分发 ==========

  /**
   * 页面注册数据回调（onLoad 时调用）
   * 回调参数: (parsedData: DeviceData)
   */
  onBLEData(callback) {
    if (typeof callback === 'function') {
      this._bleDataCallbacks.push(callback);
    }
  },

  /**
   * 页面取消注册数据回调（onUnload 时调用）
   */
  offBLEData(callback) {
    const idx = this._bleDataCallbacks.indexOf(callback);
    if (idx >= 0) this._bleDataCallbacks.splice(idx, 1);
  },

  /**
   * 页面注册 AI 异常告警回调（onLoad 时调用）
   */
  onAIAlert(callback) {
    if (typeof callback === 'function') {
      this._aiAlertCallbacks.push(callback);
    }
  },

  /**
   * 页面取消注册 AI 异常告警回调（onUnload 时调用）
   */
  offAIAlert(callback) {
    const idx = this._aiAlertCallbacks.indexOf(callback);
    if (idx >= 0) this._aiAlertCallbacks.splice(idx, 1);
  },

  /**
   * 启动中央 BLE 数据监听器（index.js 连接成功后调用一次）
   * 保持与现有 per-page listener 兼容：各页面仍可独立注册自己的监听作为 fallback
   */
  startBLECentralListener() {
    if (this._bleDataListenerReady) return;
    this._bleDataListenerReady = true;

    const { parseDeviceData, buf2Str, isAIAlertMessage } = require('./utils/ble-utils');

    wx.onBLECharacteristicValueChange((res) => {
      if (!res.value) return;
      const raw = buf2Str(res.value);
      if (!raw) return;

      // ---- NanoEdge AI 异常告警检测（最先判断，命中后直接走告警流程） ----
      if (isAIAlertMessage(raw)) {
        console.warn('⚠️ NanoEdge AI 异常告警收到: "111"');
        this._aiAlertCallbacks.forEach(cb => {
          try { cb(); } catch (e) { console.error('AI告警回调异常:', e); }
        });
        return; // 不进入正常数据解析流程
      }

      // 统一解析一次
      const data = parseDeviceData(raw);
      if (!data) return;

      // 存储到 globalData
      this.globalData.latestDeviceData = data;

      // 分发到所有订阅的页面
      this._bleDataCallbacks.forEach(cb => {
        try { cb(data, raw); } catch (e) { console.error('BLE数据回调异常:', e); }
      });
    });

    console.log('✅ BLE 中央数据监听器已启动');
  },

  // ========== 全局蓝牙断连监听 ==========

  /**
   * 启动全局蓝牙连接状态监听（在首次蓝牙连接成功后调用一次即可）
   */
  setupBLEConnectionMonitor() {
    // 防止重复注册
    if (this._bleMonitorReady) return;
    this._bleMonitorReady = true;

    wx.onBLEConnectionStateChange((res) => {
      console.log('🔌 蓝牙连接状态变化:', res.connected ? '已连接' : '已断开', res.deviceId);

      if (!res.connected) {
        // 设备断开：清除上下文 + 通知所有已注册的页面
        console.warn('⚠️ 蓝牙已断开，清除连接上下文');
        this.globalData.bleContext = null;
        this.globalData.bleConnected = false;
        wx.removeStorageSync('bleContext');

        // 通知所有注册了回调的页面
        this._bleDisconnectCallbacks.forEach(cb => {
          try { cb(res.deviceId); } catch (e) { console.error('断连回调异常:', e); }
        });
      } else {
        this.globalData.bleConnected = true;
      }
    });
  },

  /**
   * 页面注册断连回调（onLoad 时调用）
   */
  onBLEDisconnect(callback) {
    if (typeof callback === 'function') {
      this._bleDisconnectCallbacks.push(callback);
    }
  },

  /**
   * 页面取消注册断连回调（onUnload 时调用）
   */
  offBLEDisconnect(callback) {
    const idx = this._bleDisconnectCallbacks.indexOf(callback);
    if (idx >= 0) this._bleDisconnectCallbacks.splice(idx, 1);
  },

  // ========== 全局告警弹窗系统 ==========

  /**
   * 页面注册告警回调（onLoad 时调用）
   * 当有新告警时，当前活跃页面立刻弹出提示
   */
  onAlert(callback) {
    if (typeof callback === 'function') {
      this._alertCallbacks.push(callback);
    }
  },

  /**
   * 页面取消注册告警回调（onUnload 时调用）
   */
  offAlert(callback) {
    const idx = this._alertCallbacks.indexOf(callback);
    if (idx >= 0) this._alertCallbacks.splice(idx, 1);
  },

  /**
   * 添加一条告警到全局（由 dorm.js 的 evalAlerts 调用）
   */
  addPendingAlert(alert) {
    const alerts = this.globalData.pendingAlerts;
    alerts.push(alert);
    // 最多保留 20 条
    if (alerts.length > 20) alerts.splice(0, alerts.length - 20);
    wx.setStorageSync('pending_alerts', alerts);

    // 立即通知所有注册了回调的活跃页面
    this._alertCallbacks.forEach(cb => {
      try { cb(alert); } catch (e) { console.error('告警回调异常:', e); }
    });
  },

  /**
   * 弹出所有待处理告警（在 onShow 中调用）
   */
  showPendingAlerts() {
    const alerts = this.globalData.pendingAlerts;
    if (alerts.length === 0) return;

    // 取最新一条
    const latest = alerts[alerts.length - 1];
    const count = alerts.length;

    wx.showModal({
      title: `⚠️ 寝室告警`,
      content: count > 1
        ? `共有 ${count} 条未读告警\n\n最新: ${latest.message}`
        : latest.message,
      confirmText: '知道了',
      cancelText: '查看详情',
      success: (res) => {
        if (res.cancel) {
          // 跳到 dorm 查看详情
          wx.navigateTo({ url: '/pages/dorm/dorm' });
        }
        // 用户点"知道了" → 清除
        this.clearPendingAlerts();
      }
    });
  },

  /**
   * 清除所有待处理告警
   */
  clearPendingAlerts() {
    this.globalData.pendingAlerts = [];
    wx.removeStorageSync('pending_alerts');
  },

  globalData: {
    bleConnected: false,
    bleContext: null,    // { deviceId, serviceId, writeCharId, readCharId }
    userInfo: null,
    pendingAlerts: [],   // { message, type, time }
    latestDeviceData: null  // 最新设备数据对象（来自 parseDeviceData）
  }
});
