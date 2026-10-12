/**
 * 订阅消息工具模块
 * 
 * 使用流程：
 * 1. 在微信公众平台申请订阅消息模板，获取 tmplId
 * 2. 用户点击「订阅通知」→ 调用 requestSubscribe(tmplIds)
 * 3. 需要发送通知时 → 调用 sendSubscribeMessage(...)
 * 4. 发送成功后用户需再次订阅才能继续接收
 * 
 * 限制说明：
 * - 每次 wx.requestSubscribeMessage 授权仅可发送一次
 * - 用户勾选「总是保持以上选择」可免弹窗，但模板修改后仍需重新授权
 * - 发送频率限制：无官方硬性限制，但建议按需发送，避免骚扰
 */

// ==================== 模板ID 配置 ====================
// TODO: 替换为你在微信公众平台申请的真实模板ID
const TEMPLATES = {
  // 寝室异常告警模板（温度/湿度/人体等异常通知）
  // 模板字段：报警事件(thing1) / 报警地点(thing2) / 报警时间(time3)
  ALARM: 'ugQ0ANADo0lpgi_xshys_HLkf2fXrNrLO_Y2sukLLLU',
  // 设备断开通知模板（蓝牙断连提醒）—— 需要额外在公众平台添加
  DISCONNECT: 'ugQ0ANADo0lpgi_xshys_HLkf2fXrNrLO_Y2sukLLLU' // 暂复用告警模板
};

// 告警发送冷却时间（毫秒），防止频繁发送
const ALARM_COOLDOWN = 5 * 60 * 1000; // 5 分钟

// 告警冷却记录 { templateId: lastSendTimestamp }
let _cooldowns = {};

/**
 * 请求用户订阅消息授权
 * 
 * @param {string[]} tmplIds - 要订阅的模板ID列表
 * @returns {Promise<object>} 授权结果，含各模板授权状态
 * 
 * @example
 * const result = await requestSubscribe([TEMPLATES.ALARM]);
 * if (result[TEMPLATES.ALARM] === 'accept') {
 *   // 用户同意，后续可发送一次该模板消息
 * }
 */
function requestSubscribe(tmplIds) {
  return new Promise((resolve, reject) => {
    if (!tmplIds || tmplIds.length === 0) {
      reject(new Error('tmplIds 不能为空'));
      return;
    }

    wx.requestSubscribeMessage({
      tmplIds,
      success: (res) => {
        console.log('订阅消息授权结果:', res);
        // res 格式: { errMsg: "requestSubscribeMessage:ok", "xxx": "accept" }
        // 每个 tmplId 对应一个状态: 'accept' / 'reject' / 'ban'
        resolve(res);
      },
      fail: (err) => {
        console.warn('订阅消息授权失败:', err);
        reject(err);
      }
    });
  });
}

/**
 * 通过云函数发送订阅消息
 * 
 * @param {string} templateId - 模板ID
 * @param {object} data - 模板数据，格式根据模板定义
 *   eg: { thing1: { value: '温度过高' }, time2: { value: '2024-01-01 12:00' } }
 * @param {string} [page] - 点击消息跳转的页面路径
 * @returns {Promise<object>} { success, errCode?, errMsg? }
 * 
 * @example
 * const result = await sendSubscribeMessage(TEMPLATES.ALARM, {
 *   thing1: { value: '温度过高 38.5℃' },
 *   thing2: { value: '宿舍A-101' },
 *   time3: { value: '2024-06-15 14:30' }
 * });
 */
function sendSubscribeMessage(templateId, data, page) {
  return new Promise((resolve, reject) => {
    wx.cloud.callFunction({
      name: 'sendSubscribeMsg',
      data: {
        templateId,
        data,
        page: page || 'pages/dorm/dorm'
      },
      success: (res) => {
        console.log('云函数调用成功:', res);
        resolve(res.result);
      },
      fail: (err) => {
        console.error('云函数调用失败:', err);
        reject(err);
      }
    });
  });
}

/**
 * 检查告警是否在冷却期内
 * @param {string} templateId 
 * @returns {boolean} true = 冷却中，不应发送
 */
function isInCooldown(templateId) {
  const last = _cooldowns[templateId] || 0;
  return (Date.now() - last) < ALARM_COOLDOWN;
}

/**
 * 记录告警发送时间
 * @param {string} templateId 
 */
function markCooldown(templateId) {
  _cooldowns[templateId] = Date.now();
}

/**
 * 带冷却保护的告警发送
 * 仅在冷却期外且用户已授权时发送
 * 
 * @param {string} templateId 
 * @param {object} data 
 * @param {string} [page]
 */
async function sendAlarmWithCooldown(templateId, data, page) {
  if (isInCooldown(templateId)) {
    console.log('⏳ 告警冷却中，跳过发送');
    return { success: false, skipped: true, reason: 'cooldown' };
  }

  markCooldown(templateId);

  try {
    const result = await sendSubscribeMessage(templateId, data, page);
    // 如果发送成功但授权已用完（errCode 43101），不需要特殊处理
    // 下次用户主动订阅后又会恢复
    return result;
  } catch (err) {
    console.error('发送告警订阅消息失败:', err);
    return { success: false, error: err };
  }
}

module.exports = {
  TEMPLATES,
  requestSubscribe,
  sendSubscribeMessage,
  isInCooldown,
  markCooldown,
  sendAlarmWithCooldown
};
