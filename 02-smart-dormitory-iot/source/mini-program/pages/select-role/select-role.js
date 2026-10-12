const app = getApp();
const { getBLEParams } = require('../../utils/ble-utils');

Page({
  data: {
    connectedDeviceId: "",
    serviceId: "",
    writeCharId: "",
    readCharId: "",
    bleDisconnected: false,
  },

  onLoad(options) {
    const params = getBLEParams(options, ['deviceId', 'serviceId', 'writeCharId', 'readCharId']);
    this.setData({
      connectedDeviceId: params.deviceId,
      serviceId: params.serviceId,
      writeCharId: params.writeCharId,
      readCharId: params.readCharId
    });

    // 注册全局断连回调
    this._disconnectCb = (deviceId) => {
      console.warn('⚠️ select-role 页收到断连通知:', deviceId);
      this.setData({ bleDisconnected: true });
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
    app.onBLEDisconnect(this._disconnectCb);
  },

  onShow() {
    this._isVisible = true;
  },

  onHide() {
    this._isVisible = false;
  },

  onUnload() {
    if (this._disconnectCb) {
      app.offBLEDisconnect(this._disconnectCb);
      this._disconnectCb = null;
    }
  },

  // 构建 URL 参数字符串
  buildUrlParams() {
    const { connectedDeviceId, serviceId, writeCharId, readCharId } = this.data;
    return `deviceId=${connectedDeviceId}&serviceId=${serviceId}&writeCharId=${writeCharId}&readCharId=${readCharId}`;
  },

  // 选择学生模式 → 跳转现有主页
  selectStudent() {
    if (this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开", icon: "none" });
      return;
    }
    wx.navigateTo({
      url: `/pages/home/home?${this.buildUrlParams()}`
    });
  },

  // 选择宿管模式 → 跳转宿管数据页
  selectDorm() {
    if (this.data.bleDisconnected) {
      wx.showToast({ title: "蓝牙已断开", icon: "none" });
      return;
    }
    wx.navigateTo({
      url: `/pages/dorm/dorm?${this.buildUrlParams()}`
    });
  }
});
