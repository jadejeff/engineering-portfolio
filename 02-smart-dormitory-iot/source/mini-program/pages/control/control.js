const app = getApp();
const {
  buf2Str, getBLEParams, buildSendBuffer, parseDeviceData,
  CMD, DEVICE_LIGHT_KEY, DEVICE_AC_KEY, DEVICE_SOCKET_KEY, DEVICE_LOCK_KEY,
  ALARM_TEXT_MAP, ALARM_CLASS_MAP
} = require('../../utils/ble-utils');

Page({
  data: {
    // 设备状态
    lightStatus: false,
    acStatus: false,
    socketStatus: false,
    lockStatus: false,

    // BLE 参数
    connectedDeviceId: "",
    serviceId: "",
    writeCharId: "",
    readCharId: "",

    // 自定义发送
    sendText: "",
    receiveData: "",
    isSending: false,
    bleDisconnected: false,

    // 实时数据（从 JSON 回读）
    devicePower: 0,        // 总功率
    deviceTemp: "--",

    // 安全传感器（只读）
    safetySmoke: 0,        // 0=正常 1=报警
    safetyDoor: 0,         // 0=关闭 1=开启
    safetyHuman: 0,        // 0=无人 1=有人

    // 告警级别
    alarmLevel: 0,         // 0-3
    alarmText: "正常",
    alarmClass: ""
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'writeCharId', 'readCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      writeCharId: params.writeCharId,
      readCharId: params.readCharId
    });

    this.syncDeviceStates();
    this.listenBLEData();

    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ control 页收到断连通知:', deviceId);
      this.setData({ bleDisconnected: true });
      if (this._isVisible !== false) {
        wx.showModal({
          title: "蓝牙已断开",
          content: "设备连接已断开，无法发送指令",
          showCancel: false,
          success: () => { wx.navigateBack({ delta: 2 }); }
        });
      }
    };
    getApp().onBLEDisconnect(this._disconnectCb);
  },

  onShow() {
    this._isVisible = true;
    this.syncDeviceStates();
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
    if (this._toggleTimers) {
      Object.values(this._toggleTimers).forEach(t => clearTimeout(t));
      this._toggleTimers = null;
    }
  },

  // ============================================================
  //  状态同步（从 storage 恢复）
  // ============================================================
  syncDeviceStates() {
    try {
      this.setData({
        lightStatus:  !!wx.getStorageSync(DEVICE_LIGHT_KEY),
        acStatus:     !!wx.getStorageSync(DEVICE_AC_KEY),
        socketStatus: !!wx.getStorageSync(DEVICE_SOCKET_KEY),
        lockStatus:   !!wx.getStorageSync(DEVICE_LOCK_KEY)
      });
    } catch (e) { /* ignore */ }
  },

  // ============================================================
  //  通用发送方法 — 2 字节协议
  // ============================================================
  sendCommand(cmdArray, options) {
    if (this.checkDisconnected()) return;
    if (this.data.isSending) return;

    const { connectedDeviceId, serviceId, writeCharId } = this.data;
    if (!connectedDeviceId) {
      wx.showToast({ title: "未连接设备", icon: "none" });
      return;
    }

    this.setData({ isSending: true });
    wx.showLoading({ title: "发送中..." });

    const buffer = buildSendBuffer(cmdArray);

    console.log('🔍 [sendCommand] BLE write:', cmdArray.map(b => '0x' + b.toString(16).padStart(2, '0')).join(' '));

    wx.writeBLECharacteristicValue({
      deviceId: connectedDeviceId,
      serviceId: serviceId,
      characteristicId: writeCharId,
      value: buffer,
      success: () => {
        console.log('✅ [sendCommand] write success');
        wx.hideLoading();
        this.setData({ isSending: false });
        if (options && options.success) options.success();
      },
      fail: (err) => {
        console.error('❌ [sendCommand] write fail:', JSON.stringify(err));
        wx.hideLoading();
        this.setData({ isSending: false });
        wx.showToast({ title: "发送失败", icon: "error" });
        if (options && options.fail) options.fail(err);
      }
    });
  },

  checkDisconnected() {
    if (this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开", icon: "none" });
      return true;
    }
    return false;
  },

  // ============================================================
  //  通用 Toggle 切换（通过 WXML data-* 属性驱动，含 3s 超时回滚）
  // ============================================================
  toggleDevice(e) {
    if (this.checkDisconnected()) return;

    const ds = e.currentTarget.dataset;
    const device    = ds.device;            // e.g. "light"
    const cmdOn     = CMD[ds.cmdOn];        // e.g. CMD.LED_ON
    const cmdOff    = CMD[ds.cmdOff];       // e.g. CMD.LED_OFF
    const statusKey = device + 'Status';     // e.g. lightStatus

    const oldState = this.data[statusKey];
    const newState = !oldState;
    const cmd = newState ? cmdOn : cmdOff;

    // 序列号防竞态：每次切换递增，超时回滚时仅匹配的序列号才生效
    if (!this._toggleSeq) this._toggleSeq = 0;
    const toggleId = ++this._toggleSeq;

    // 清除旧超时定时器
    if (this._toggleTimers) {
      clearTimeout(this._toggleTimers[device]);
    } else {
      this._toggleTimers = {};
    }

    this.sendCommand(cmd, {
      success: () => {
        const update = {};
        update[statusKey] = newState;
        this.setData(update);

        // 超时回滚：3 秒内未收到 JSON 确认则自动恢复旧状态
        this._toggleTimers[device] = setTimeout(() => {
          // 仅当序列号匹配时才回滚（防止快速切换时的错误回滚）
          if (this._toggleSeq !== toggleId) return;
          const revert = {};
          revert[statusKey] = oldState;
          this.setData(revert);
          wx.showToast({ title: '设备未响应,已回滚', icon: 'none' });
        }, 3000);
      }
    });
  },

  // ============================================================
  //  一键全关
  // ============================================================
  allOff() {
    if (this.checkDisconnected()) return;

    wx.showModal({
      title: "一键全关",
      content: "将关闭灯、空调、插座并锁定门锁，确定继续？",
      confirmText: "确定关闭",
      confirmColor: "#EF4444",
      success: (res) => {
        if (!res.confirm) return;

        // 依次发送4条关闭指令
        const cmds = [CMD.LED_OFF, CMD.AC_OFF, CMD.SOCKET_OFF, CMD.LOCK_ON];
        this._sendSequence(cmds, 0, () => {
          this.setData({
            lightStatus: false,
            acStatus: false,
            socketStatus: false,
            lockStatus: true
          });
          wx.setStorageSync(DEVICE_LIGHT_KEY, false);
          wx.setStorageSync(DEVICE_AC_KEY, false);
          wx.setStorageSync(DEVICE_SOCKET_KEY, false);
          wx.setStorageSync(DEVICE_LOCK_KEY, true);
          wx.showToast({ title: "已全部关闭", icon: "success" });
        });
      }
    });
  },

  // 顺序发送多条指令（带中途断连检测）
  _sendSequence(cmds, index, onDone) {
    if (index >= cmds.length) { onDone(); return; }

    const { connectedDeviceId, serviceId, writeCharId } = this.data;

    // 中途断连检测：停止发送并跳过 onDone，避免 UI 被错误设为全关状态
    if (!connectedDeviceId || this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开，全关中断", icon: "none" });
      return;
    }

    const buffer = buildSendBuffer(cmds[index]);

    wx.writeBLECharacteristicValue({
      deviceId: connectedDeviceId,
      serviceId: serviceId,
      characteristicId: writeCharId,
      value: buffer,
      success: () => {
        // 200ms 间隔发送下一条
        setTimeout(() => {
          this._sendSequence(cmds, index + 1, onDone);
        }, 200);
      },
      fail: () => {
        // 失败也继续发送下一条
        setTimeout(() => {
          this._sendSequence(cmds, index + 1, onDone);
        }, 200);
      }
    });
  },

  // ============================================================
  //  自定义指令
  // ============================================================
  onInput(e) {
    this.setData({ sendText: e.detail.value });
  },

  sendCustom() {
    if (this.checkDisconnected()) return;
    if (this.data.isSending) return;

    let text = this.data.sendText;
    if (!text) {
      wx.showToast({ title: "请输入内容", icon: "none" });
      return;
    }

    if (!/^[\x20-\x7E]+$/.test(text)) {
      wx.showToast({ title: "仅支持英文和标点", icon: "none" });
      return;
    }

    const { connectedDeviceId, serviceId, writeCharId } = this.data;
    if (!connectedDeviceId) {
      wx.showToast({ title: "未连接设备", icon: "none" });
      return;
    }

    this.setData({ isSending: true });
    wx.showLoading({ title: "发送中..." });

    const buffer = buildSendBuffer(text);

    wx.writeBLECharacteristicValue({
      deviceId: connectedDeviceId,
      serviceId: serviceId,
      characteristicId: writeCharId,
      value: buffer,
      success: () => {
        wx.hideLoading();
        this.setData({ isSending: false, sendText: "" });
        wx.showToast({ title: "已发送" });
      },
      fail: () => {
        wx.hideLoading();
        this.setData({ isSending: false });
        wx.showToast({ title: "发送失败", icon: "error" });
      }
    });
  },

  // ============================================================
  //  BLE 数据监听 — 解析 JSON 回读状态
  // ============================================================
  listenBLEData() {
    const ctx = getApp().globalData.bleContext || {};
    const deviceId = this.data.connectedDeviceId || ctx.deviceId;
    const serviceId = this.data.serviceId || ctx.serviceId;
    const readCharId = this.data.readCharId || ctx.readCharId;

    if (!deviceId || !serviceId || !readCharId) {
      console.warn('⚠️ listenBLEData 缺少参数');
      return;
    }

    // notify 已在 index.js 中统一开启，此处不再重复调用

    // 通过中央分发器订阅数据
    this._bleDataCb = (data, raw) => {
      console.log('📥 control收到:', raw);
      this.setData({ receiveData: raw });

      // 从 JSON 回读设备状态
      this.setData({
        lightStatus:  !!data.light,
        acStatus:     !!data.ac,
        socketStatus: !!data.socket,
        lockStatus:   !!data.lock,
        devicePower:  data.power.toFixed(1),
        deviceTemp:   data.temp.toFixed(1),
        safetySmoke:  data.smoke,
        safetyDoor:   data.door,
        safetyHuman:  data.human,
        alarmLevel:   data.alarm,
        alarmText:    ALARM_TEXT_MAP[data.alarm] || '--',
        alarmClass:   ALARM_CLASS_MAP[data.alarm] || ''
      });

      wx.setStorageSync(DEVICE_LIGHT_KEY, !!data.light);
      wx.setStorageSync(DEVICE_AC_KEY, !!data.ac);
      wx.setStorageSync(DEVICE_SOCKET_KEY, !!data.socket);
      wx.setStorageSync(DEVICE_LOCK_KEY, !!data.lock);

      // 取消超时回滚定时器
      if (this._toggleTimers) {
        Object.values(this._toggleTimers).forEach(t => clearTimeout(t));
        this._toggleTimers = {};
      }
    };
    app.onBLEData(this._bleDataCb);

    // Fallback：如果中央分发器未启动，启用备用监听
    if (!app._bleDataListenerReady) {
      console.warn('⚠️ 中央分发器未启动，启用备用监听');
      this._charChangeCb = (res) => {
        if (!res.value) return;
        let str = buf2Str(res.value);
        this.setData({ receiveData: str });
        const data = parseDeviceData(str);
        if (data && this._bleDataCb) this._bleDataCb(data, str);
      };
      wx.onBLECharacteristicValueChange(this._charChangeCb);
    }
  }
});
