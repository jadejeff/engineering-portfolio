const app = getApp();
const {
  buf2Str, getBLEParams, buildSendBuffer, parseDeviceData,
  CMD, DEVICE_SCENE_KEY, DEVICE_SYS_MODE_KEY, DEVICE_LIGHT_KEY
} = require('../../utils/ble-utils');

Page({
  data: {
    connectedDeviceId: "",
    serviceId: "",
    writeCharId: "",
    readCharId: "",
    currentScene: "",        // 当前场景模式: sleep | study | rest
    currentSysMode: "",      // 当前系统模式: normal | eco | safe
    deviceMode: "",          // 设备实际返回的模式文本
    isSending: false,
    bleDisconnected: false,
    dataSynced: false        // 是否已从设备同步过模式
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'writeCharId', 'readCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      writeCharId: params.writeCharId,
      readCharId: params.readCharId
    });

    // 从 storage 恢复上次选中的模式
    const savedScene = wx.getStorageSync(DEVICE_SCENE_KEY) || '';
    const savedSysMode = wx.getStorageSync(DEVICE_SYS_MODE_KEY) || '';
    this.setData({
      currentScene: savedScene,
      currentSysMode: savedSysMode
    });

    // 监听 BLE 数据以获取设备实际模式
    this.listenDeviceData();

    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ scene 页收到断连通知:', deviceId);
      this.setData({ bleDisconnected: true });
      if (this._isVisible !== false) {
        wx.showModal({
          title: "蓝牙已断开",
          content: "设备连接已断开，无法发送场景指令",
          showCancel: false,
          success: () => { wx.navigateBack({ delta: 2 }); }
        });
      }
    };
    app.onBLEDisconnect(this._disconnectCb);
  },

  onShow() {
    this._isVisible = true;
    // 从 Storage 恢复场景/系统模式（防止其他页面修改后此处过时）
    const savedScene = wx.getStorageSync(DEVICE_SCENE_KEY) || '';
    const savedSysMode = wx.getStorageSync(DEVICE_SYS_MODE_KEY) || '';
    if (savedScene !== this.data.currentScene || savedSysMode !== this.data.currentSysMode) {
      this.setData({
        currentScene: savedScene,
        currentSysMode: savedSysMode
      });
    }
  },

  onHide() {
    this._isVisible = false;
  },

  onUnload() {
    if (this._sceneRollbackTimer) {
      clearTimeout(this._sceneRollbackTimer);
      this._sceneRollbackTimer = null;
    }
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
  },

  // ============================================================
  //  监听 BLE 数据 — 从设备回读模式（JSON m 字段）
  // ============================================================
  listenDeviceData() {
    const ctx = app.globalData.bleContext || {};
    const deviceId = this.data.connectedDeviceId || ctx.deviceId;
    const serviceId = this.data.serviceId || ctx.serviceId;
    const readCharId = this.data.readCharId || ctx.readCharId;

    if (!deviceId || !serviceId || !readCharId) {
      console.warn('⚠️ scene 缺少蓝牙参数，无法监听回读');
      return;
    }

    // notify 已在 index.js 中统一开启，此处不再重复调用

    // 通过中央分发器订阅 BLE 数据
    this._bleDataCb = (data) => {
      if (!data) return;

      // 从 JSON m 字段同步模式到页面显示
      const modeMap = { 0: '', 1: 'sleep', 2: 'dorm' };
      const modeTextMap = { 0: '正常模式', 1: '睡眠', 2: '宿管' };

      const deviceSceneMode = modeMap[data.mode] || '';
      const deviceModeText = modeTextMap[data.mode] || '--';

      if (deviceSceneMode && deviceSceneMode !== this.data.currentScene) {
        this.setData({
          currentScene: deviceSceneMode,
          deviceMode: deviceModeText,
          dataSynced: true
        });
        wx.setStorageSync(DEVICE_SCENE_KEY, deviceSceneMode);
        // BLE 回读确认，清除滚动定时器
        this._clearSceneRollback();
      } else {
        this.setData({
          deviceMode: deviceModeText,
          dataSynced: true
        });
        // 即使模式未变，也清除回滚定时器（表示设备已响应）
        this._clearSceneRollback();
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
        const data = parseDeviceData(raw);
        if (data && this._bleDataCb) this._bleDataCb(data);
      };
      wx.onBLECharacteristicValueChange(this._charChangeCb);
    }
  },

  // ============================================================
  //  通用发送方法 — 2 字节协议
  // ============================================================
  sendCommand(cmdArray, onSuccess) {
    if (this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开", icon: "none" });
      return;
    }
    if (this.data.isSending) return;

    const { connectedDeviceId, serviceId, writeCharId } = this.data;
    if (!connectedDeviceId || !writeCharId) {
      wx.showToast({ title: "蓝牙未连接", icon: "none" });
      return;
    }

    this.setData({ isSending: true });

    const buffer = buildSendBuffer(cmdArray);

    console.log('🔍 [scene] BLE write:', cmdArray.map(b => '0x' + b.toString(16).padStart(2, '0')).join(' '));

    wx.writeBLECharacteristicValue({
      deviceId: connectedDeviceId,
      serviceId: serviceId,
      characteristicId: writeCharId,
      value: buffer,
      success: () => {
        console.log('✅ [scene] write success');
        this.setData({ isSending: false });
        wx.showToast({ title: "指令已发送", icon: "success" });
        if (onSuccess) onSuccess();
      },
      fail: (err) => {
        this.setData({ isSending: false });
        console.error('❌ [scene] write fail:', JSON.stringify(err));
        wx.showToast({ title: "发送失败", icon: "error" });
      }
    });
  },

  // ============================================================
  //  场景模式（带 3s 超时回滚）
  // ============================================================
  _clearSceneRollback() {
    if (this._sceneRollbackTimer) {
      clearTimeout(this._sceneRollbackTimer);
      this._sceneRollbackTimer = null;
    }
  },

  _applySceneMode(cmd, sceneKey, extraStorageFn) {
    const prevScene = this.data.currentScene;
    this._clearSceneRollback();

    this.sendCommand(cmd, () => {
      this.setData({ currentScene: sceneKey });
      wx.setStorageSync(DEVICE_SCENE_KEY, sceneKey);
      if (extraStorageFn) extraStorageFn();

      // 3s 超时回滚：若 BLE 未确认新模式则回退
      this._sceneRollbackTimer = setTimeout(() => {
        if (this.data.currentScene === sceneKey) {
          this.setData({ currentScene: prevScene });
          wx.setStorageSync(DEVICE_SCENE_KEY, prevScene);
          wx.showToast({ title: '设备未响应，已回滚', icon: 'none' });
        }
      }, 3000);
    });
  },

  sleepMode() {
    this._applySceneMode(CMD.SCENE_SLEEP, "sleep", () => {
      wx.setStorageSync(DEVICE_LIGHT_KEY, false);
    });
  },

  studyMode() {
    this._applySceneMode(CMD.SCENE_STUDY, "study", () => {
      wx.setStorageSync(DEVICE_LIGHT_KEY, true);
    });
  },

  restMode() {
    this._applySceneMode(CMD.SCENE_REST, "rest", () => {
      wx.setStorageSync(DEVICE_LIGHT_KEY, false);
    });
  },

  // ============================================================
  //  系统模式（带 3s 超时回滚）
  // ============================================================
  _applySysMode(cmd, modeKey) {
    const prevMode = this.data.currentSysMode;
    this._clearSceneRollback();

    this.sendCommand(cmd, () => {
      this.setData({ currentSysMode: modeKey });
      wx.setStorageSync(DEVICE_SYS_MODE_KEY, modeKey);

      // 3s 超时回滚
      this._sceneRollbackTimer = setTimeout(() => {
        if (this.data.currentSysMode === modeKey) {
          this.setData({ currentSysMode: prevMode });
          wx.setStorageSync(DEVICE_SYS_MODE_KEY, prevMode);
          wx.showToast({ title: '设备未响应，已回滚', icon: 'none' });
        }
      }, 3000);
    });
  },

  normalMode() {
    this._applySysMode(CMD.MODE_NORMAL, "normal");
  },

  ecoMode() {
    this._applySysMode(CMD.MODE_ECO, "eco");
  },

  safeMode() {
    this._applySysMode(CMD.MODE_SAFE, "safe");
  }
});
