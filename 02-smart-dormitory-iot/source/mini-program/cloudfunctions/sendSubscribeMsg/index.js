/**
 * 发送订阅消息云函数
 * 
 * 前置条件：
 * 1. 微信公众平台 → 功能 → 订阅消息 → 选用模板（获取模板ID）
 * 2. 云函数 config.json 中声明 openapi 权限
 * 3. 用户已通过 wx.requestSubscribeMessage 授权
 * 
 * 每个用户每次授权仅可发送一次，发送后需重新授权
 */

const cloud = require('wx-server-sdk');
cloud.init({ env: cloud.DYNAMIC_CURRENT_ENV });

exports.main = async (event, context) => {
  const { templateId, data, page } = event;
  const { OPENID } = cloud.getWXContext();

  if (!templateId || !data) {
    return { success: false, errMsg: '缺少 templateId 或 data 参数' };
  }

  try {
    const result = await cloud.openapi.subscribeMessage.send({
      touser: OPENID,
      templateId,
      page: page || 'pages/dorm/dorm',
      data,
      // TODO: 上线前改为 'formal'
      miniprogramState: 'developer'
    });
    console.log('订阅消息发送成功:', result);
    return { success: true, result };
  } catch (err) {
    console.error('订阅消息发送失败:', err.errCode, err.errMsg);
    return { success: false, errCode: err.errCode, errMsg: err.errMsg };
  }
};
