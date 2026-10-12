const { getBLEParams } = require('../../utils/ble-utils');

Page({
  data: {
    connectedDeviceId: "",
    serviceId: "",
    writeCharId: "",
    readCharId: ""
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'writeCharId', 'readCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      writeCharId: params.writeCharId,
      readCharId: params.readCharId
    });

    // ✅ 注册全局断连回调
    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ home 页收到断连通知:', deviceId);
      // 只在当前页面可视时弹窗
      if (this._isVisible !== false) {
        wx.showModal({
          title: "蓝牙已断开",
          content: "设备连接已断开，请重新连接",
          showCancel: false,
          success: () => {
            wx.navigateBack();
          }
        });
      }
    };
    getApp().onBLEDisconnect(this._disconnectCb);
  },

  onShow() {
    this._isVisible = true;
  },

  onHide() {
    this._isVisible = false;
  },

  onUnload() {
    // 取消注册断连回调
    if (this._disconnectCb) {
      getApp().offBLEDisconnect(this._disconnectCb);
      this._disconnectCb = null;
    }
  },

  goToMonitor() {
    wx.navigateTo({
      url: `/pages/monitor/monitor?deviceId=${this.data.connectedDeviceId}&serviceId=${this.data.serviceId}&writeCharId=${this.data.writeCharId}&readCharId=${this.data.readCharId}`
    });
  },

  goToScene() {
    wx.navigateTo({
      url: `/pages/scene/scene?deviceId=${this.data.connectedDeviceId}&serviceId=${this.data.serviceId}&writeCharId=${this.data.writeCharId}&readCharId=${this.data.readCharId}`
    });
  },

  goToControl() {
    wx.navigateTo({
      url: `/pages/control/control?deviceId=${this.data.connectedDeviceId}&serviceId=${this.data.serviceId}&writeCharId=${this.data.writeCharId}&readCharId=${this.data.readCharId}`
    });
  },

  // 手动断开按钮
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
        // ✅ 使用 globalData 传递状态
        const app = getApp();
        app.globalData.bleContext = null;
        app.globalData.bleConnected = false;
        wx.removeStorageSync('bleContext');

        wx.showToast({ title: "已断开连接" });
        setTimeout(() => {
          wx.navigateBack();
        }, 500);
      },
      fail: (err) => {
        console.error('❌ 断开失败:', err);
        wx.showToast({ title: "断开失败", icon: "error" });
      }
    });
  }
});