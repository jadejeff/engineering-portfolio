Page({
  data: {
    devices: [],
    connectedDeviceId: "",
    serviceId: "",
    writeCharId: "",
    readCharId: "",  // 用来接收单片机数据
    isScanning: false,
    isConnected: false,
  },

  onLoad() {
    this.openBluetoothAdapter();
  },

  onShow() {
    // ✅ 从子页面 navigateBack 返回时（断连等情况），检测并重置状态
    if (!getApp().globalData.bleConnected && this.data.isConnected) {
      this.resetConnectionState();
    }
  },

  // ========== 取消连接 / 重置状态 ==========

  cancelConnection() {
    const deviceId = this.data.connectedDeviceId;
    wx.showModal({
      title: "取消连接",
      content: "确定要取消当前连接吗？",
      success: (res) => {
        if (res.confirm) {
          if (deviceId) {
            wx.closeBLEConnection({ deviceId, fail: () => {} });
          }
          this.resetConnectionState();
        }
      }
    });
  },

  resetConnectionState() {
    wx.hideLoading();
    // 同步清理 globalData，防止 onShow 检测断连时状态不一致
    const app = getApp();
    app.globalData.bleConnected = false;
    app.globalData.bleContext = null;
    wx.removeStorageSync('bleContext');
    this.setData({
      isConnected: false,
      connectedDeviceId: "",
      serviceId: "",
      writeCharId: "",
      readCharId: "",
    });
  },

  // ✅ 根据 RSSI 计算信号等级 (1-4)
  calcSignalLevel(rssi) {
    if (rssi >= -55) return 4;       // 极强
    if (rssi >= -70) return 3;       // 良好
    if (rssi >= -85) return 2;       // 一般
    return 1;                         // 较弱
  },

  onUnload() {
    // ✅ 清理扫描资源 + BLE 监听，不关闭蓝牙适配器（子页面可能仍在使用）
    this.stopScan();
    if (this._charChangeCb) {
      wx.offBLECharacteristicValueChange(this._charChangeCb);
      this._charChangeCb = null;
    }
  },

  // ========== 蓝牙适配器初始化 ==========

  openBluetoothAdapter() {
    wx.openBluetoothAdapter({
      success: () => {
        console.log("✅ 蓝牙初始化成功");
        // 检查蓝牙状态
        wx.getBluetoothAdapterState({
          success: (state) => {
            console.log('蓝牙适配器状态:', state.available ? '可用' : '不可用',
                        state.discovering ? '(正在扫描中)' : '');
          }
        });
      },
      fail: (err) => {
        console.error('❌ 蓝牙初始化失败:', err);
        wx.showModal({
          title: "蓝牙初始化失败",
          content: "请开启手机蓝牙和定位服务后重试\n\n(" + (err.errMsg || '未知错误') + ")",
          showCancel: false,
          confirmText: "我知道了"
        });
      },
    });
  },

  // ========== 设备扫描 ==========

  startScanDevices() {
    if (this.data.isScanning) return;

    // 先检查蓝牙适配器是否可用
    wx.getBluetoothAdapterState({
      success: (state) => {
        if (!state.available) {
          wx.showModal({
            title: "蓝牙未开启",
            content: "请先开启手机蓝牙",
            showCancel: false
          });
          return;
        }
        this.doStartScan();
      },
      fail: () => {
        this.doStartScan(); // 兜底：状态查询失败也不阻止扫描
      }
    });
  },

  doStartScan() {
    // ✅ 先移除旧监听，防止重复注册
    wx.offBluetoothDeviceFound();

    this.setData({ isScanning: true, devices: [] });
    wx.showLoading({ title: "扫描中..." });

    wx.startBluetoothDevicesDiscovery({
      services: [],
      allowDuplicatesKey: false,
      success: () => {
        console.log('🔍 蓝牙扫描已启动');
      },
      fail: (err) => {
        wx.hideLoading();
        this.setData({ isScanning: false });
        console.error('❌ 启动扫描失败:', err);
        wx.showModal({
          title: "扫描失败",
          content: "无法启动蓝牙扫描，请检查蓝牙和定位权限\n\n(" + (err.errMsg || '未知错误') + ")",
          showCancel: false
        });
      }
    });

    wx.onBluetoothDeviceFound((res) => {
      let devices = this.data.devices;
      res.devices.forEach(device => {
        if (!device.name) return;
        const has = devices.some(d => d.deviceId === device.deviceId);
        if (!has) {
          // ✅ 计算 RSSI 信号等级（1-4），并排序插入
          device._signalLevel = this.calcSignalLevel(device.RSSI);
          devices.push(device);
        }
      });
      // ✅ 按信号强度降序排列
      devices.sort((a, b) => (b.RSSI || -100) - (a.RSSI || -100));
      this.setData({ devices });
    });

    // 8 秒后自动停止扫描
    this._scanTimer = setTimeout(() => {
      this.stopScan();
      if (this.data.devices.length === 0) {
        wx.showToast({ title: "未发现设备", icon: "none" });
      }
    }, 8000);
  },

  stopScan() {
    if (this._scanTimer) {
      clearTimeout(this._scanTimer);
      this._scanTimer = null;
    }
    wx.stopBluetoothDevicesDiscovery({
      fail: () => {} // 静默失败
    });
    wx.offBluetoothDeviceFound();
    wx.hideLoading();
    this.setData({ isScanning: false });
  },

  // ========== 设备连接（带超时） ==========

  connectDevice(e) {
    if (this.data.isConnected) return;

    const deviceId = e.currentTarget.dataset.deviceid;
    if (!deviceId) {
      wx.showToast({ title: "设备ID无效", icon: "none" });
      return;
    }

    wx.showLoading({ title: "连接中..." });

    // 停止扫描后再连接
    this.stopScan();

    // 检查蓝牙适配器状态
    wx.getBluetoothAdapterState({
      complete: () => {
        this.doConnect(deviceId);
      }
    });
  },

  doConnect(deviceId) {
    const CONNECT_TIMEOUT = 10000; // 连接超时 10 秒

    // Promise.race 实现超时控制
    const connectPromise = new Promise((resolve, reject) => {
      wx.createBLEConnection({
        deviceId,
        timeout: CONNECT_TIMEOUT,
        success: (res) => resolve(res),
        fail: (err) => reject(err),
      });
    });

    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => reject(new Error('连接超时')), CONNECT_TIMEOUT);
    });

    Promise.race([connectPromise, timeoutPromise])
      .then(() => {
        console.log('✅ 蓝牙连接成功:', deviceId);
        this.setData({
          connectedDeviceId: deviceId,
          isConnected: true,
        });
        wx.hideLoading();
        wx.showToast({ title: "连接成功" });

        // ✅【关键】首次连接成功后，启动全局断连监听
        getApp().setupBLEConnectionMonitor();
        getApp().globalData.bleConnected = true;

        // 设置 MTU
        this.setupMTU(deviceId);
      })
      .catch((err) => {
        wx.hideLoading();
        const msg = (err && err.errMsg) || err.message || '连接失败';
        console.error('❌ 蓝牙连接失败:', msg);
        wx.showModal({
          title: "连接失败",
          content: "请确保设备在附近且已上电\n\n(" + msg + ")",
          showCancel: false
        });
        // 连接失败时断开以清理状态
        wx.closeBLEConnection({ deviceId, fail: () => {} });
      });
  },

  // ========== MTU 协商（带超时） ==========

  setupMTU(deviceId) {
    const MTU_TIMEOUT = 3000;

    const mtuPromise = new Promise((resolve) => {
      wx.setBLEMTU({
        deviceId,
        mtu: 512,
        success: (mtuRes) => {
          console.log('✅ MTU 协商成功:', mtuRes.mtu);
          resolve();
        },
        fail: (err) => {
          console.warn('⚠️ MTU 协商失败（使用默认20字节）:', err);
          resolve(); // MTU 失败不阻塞流程
        },
      });
    });

    Promise.race([mtuPromise, new Promise(r => setTimeout(r, MTU_TIMEOUT))])
      .then(() => {
        this.getService(deviceId);
      });
  },

  // ========== 获取服务和特征（带超时） ==========

  getService(deviceId) {
    const SERVICE_TIMEOUT = 5000;

    const svcPromise = new Promise((resolve, reject) => {
      wx.getBLEDeviceServices({
        deviceId,
        success: (res) => resolve(res),
        fail: (err) => reject(err),
      });
    });

    Promise.race([svcPromise, new Promise((_, r) => setTimeout(() => r(new Error('获取服务超时')), SERVICE_TIMEOUT))])
      .then((res) => {
        const ser = res.services.find(s => s.isPrimary);
        if (ser) {
          console.log('✅ 找到主服务:', ser.uuid);
          this.setData({ serviceId: ser.uuid });
          this.getChar(deviceId, ser.uuid);
        } else {
          wx.hideLoading();
          this.resetConnectionState();
          wx.showModal({
            title: "未找到服务",
            content: "设备未提供主服务，请检查固件",
            showCancel: false
          });
        }
      })
      .catch((err) => {
        wx.hideLoading();
        this.resetConnectionState();
        console.error('❌ 获取服务失败:', err.errMsg || err.message);
        wx.showModal({
          title: "获取服务失败",
          content: "请重试连接\n\n(" + (err.errMsg || err.message) + ")",
          showCancel: false
        });
      });
  },

  getChar(deviceId, serviceId) {
    const CHAR_TIMEOUT = 5000;

    const charPromise = new Promise((resolve, reject) => {
      wx.getBLEDeviceCharacteristics({
        deviceId, serviceId,
        success: (res) => resolve(res),
        fail: (err) => reject(err),
      });
    });

    Promise.race([charPromise, new Promise((_, r) => setTimeout(() => r(new Error('获取特征超时')), CHAR_TIMEOUT))])
      .then((res) => {
        let chars = res.characteristics;

        // 1. 找到可写特征（手机→单片机）
        let writeChar = chars.find(c =>
          c.properties.write || c.properties.writeNoResponse
        );

        // 2. 找到 notify 特征（单片机→手机）
        let readChar = chars.find(c => c.properties.notify);

        if (!writeChar || !readChar) {
          wx.hideLoading();
          this.resetConnectionState();
          wx.showModal({
            title: "特征不完整",
            content: `可写特征: ${writeChar ? '✅' : '❌'}\n通知特征: ${readChar ? '✅' : '❌'}\n请检查固件`,
            showCancel: false
          });
          return;
        }

        console.log('✅ writeChar:', writeChar.uuid, '| readChar:', readChar.uuid);

        // 保存两个特征
        this.setData({
          writeCharId: writeChar.uuid,
          readCharId: readChar.uuid
        }, () => {
          // ✅ 保存 BLE 上下文到全局，供所有页面读取
          const ctx = {
            deviceId: deviceId,
            serviceId: serviceId,
            writeCharId: writeChar.uuid,
            readCharId: readChar.uuid
          };
          getApp().globalData.bleContext = ctx;
          wx.setStorageSync('bleContext', ctx);

          // ✅ 开启通知
          this.enableNotify(deviceId, serviceId, readChar.uuid);
        });
      })
      .catch((err) => {
        wx.hideLoading();
        this.resetConnectionState();
        console.error('❌ 获取特征失败:', err.errMsg || err.message);
        wx.showModal({
          title: "获取特征失败",
          content: "请重试连接\n\n(" + (err.errMsg || err.message) + ")",
          showCancel: false
        });
      });
  },

  // ========== 开启通知并跳转 ==========

  enableNotify(deviceId, serviceId, charId) {
    wx.notifyBLECharacteristicValueChange({
      deviceId,
      serviceId,
      characteristicId: charId,
      state: true,
      success: () => {
        console.log("✅ 通知开启成功！单片机可以发数据了！");

        // ✅ 启动 BLE 中央数据分发器（仅此一处注册全局监听）
        getApp().startBLECentralListener();

        wx.hideLoading();

        // 跳转角色选择页
        setTimeout(() => {
          // 清理 index 页的 BLE 监听，避免无效回调分发
          if (this._charChangeCb) {
            wx.offBLECharacteristicValueChange(this._charChangeCb);
            this._charChangeCb = null;
          }
          wx.navigateTo({
            url: `/pages/select-role/select-role?deviceId=${this.data.connectedDeviceId}` +
                 `&serviceId=${this.data.serviceId}` +
                 `&writeCharId=${this.data.writeCharId}` +
                 `&readCharId=${this.data.readCharId}`
          });
        }, 500);
      },
      fail: (err) => {
        wx.hideLoading();
        console.error('❌ 开启通知失败:', err);
        wx.showModal({
          title: "通知开启失败",
          content: "设备可能无法主动发送数据，但不影响控制功能",
          cancelText: "返回重试",
          confirmText: "继续使用",
          success: (res) => {
          if (res.confirm) {
            // 用户选择继续使用 → 跳转角色选择页
            setTimeout(() => {
              // 清理 index 页的 BLE 监听
              if (this._charChangeCb) {
                wx.offBLECharacteristicValueChange(this._charChangeCb);
                this._charChangeCb = null;
              }
              wx.navigateTo({
                url: `/pages/select-role/select-role?deviceId=${this.data.connectedDeviceId}` +
                     `&serviceId=${this.data.serviceId}` +
                     `&writeCharId=${this.data.writeCharId}` +
                     `&readCharId=${this.data.readCharId}`
              });
            }, 300);
            } else {
              // 用户选择返回重试
              this.resetConnectionState();
            }
          }
        });
      }
    });
  },
});