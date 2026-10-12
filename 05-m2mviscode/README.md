# M2MVisCode

面向机器生成与机器识别场景的随机视觉编码与序列识别研究。本页作为科研项目案例（Research Case Study）展示。

## 研究内容

- 使用 Key-conditioned stochastic visual encoding 生成可复现且带有受控随机变化的视觉编码。
- 从完整图像识别符号序列，不依赖字符分割；识别骨干采用卷积与循环网络，并通过 CTC 训练和解码。
- 接入多个公开 baseline，按统一协议比较不同识别路线，并隔离 validation 与 protected test。

## 我的工作

作为论文第一作者，我负责方法设计与实现、实验协议设计、公开 baseline 接入与统一评测、validation / protected test 隔离、模型和结果冻结与审计，以及论文撰写。

## 论文

[查看 M2MVisCode 论文 PDF](Guo_M2MVisCode.pdf)｜已投稿，审稿中。

## 说明

因论文审稿及相关知识产权工作仍在进行，当前仅公开研究概览与论文材料，不提供完整训练代码、模型权重及实验配置。
