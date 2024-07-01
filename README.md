# 2023年中国大学生工程实践与创新能力大赛总结与改进
## 仓库内容
这里主要开源我们参加工创赛所有的文件，包括机械，主控，视觉，电路板，各种时期的代码都有，方案包含数字舵机、串行总线舵机、激光定位、视觉定位、初赛、决赛等全面内容。


----------

***这里代码量太多了，为了能够提交仓库，除了国赛文件夹里的代码，其他STM32代码的Drivers文件夹内容均通过bat脚本删了，需要查看老版本并代码编译的，可以复制国赛文件夹里的Drivers，用于编译。*** 

## 概述
记录一下我们biubiu小队参加2023年中国大学生工程实践与创新能力大赛“智能+物流搬运”赛项的过程，然后对比赛中发生的问题做个记录和总结，并传递一下我们对这个赛项的经验与坑。
主要包括：
> [历程分享——我们的参赛经历，过程艰辛，跌宕起伏，几经反转；](https://www.cnblogs.com/sparkle-now/p/18277263/gong-chuang-sai-zong-jie-yu-zhan-wanggai-shu)

> 比赛解读——主要讲一下我们对这个规则的想法；

> [选型经验——主要讲做比赛选元件，用东西的坑和经验](https://www.cnblogs.com/sparkle-now/p/18278041/gong-chuang-sai-zong-jie-yu-gai-jinxuan-xing-jian)

> 方案建议——主要说我们队的方案和改进，供大家参考；

> 队长经验——作为队长，讲一下比赛中团队管理，项目进展推进，人员管理之类的；

> 电控建议——分享一下比赛中电控选手的注意事项；

> 结构建议——分享一下比赛中负责机械部分的内容；

> 硬件建议——分享一下比赛中电路板设计的注意事项；

> 视觉建议——分享一下物料识别，色环识别，以及补光；

> 决赛分享——决赛就是团队实力的综合体现；

## 国赛初赛视频分享

[工创赛国赛初赛视频bilibili](https://www.bilibili.com/video/BV1Wj411s7hJ/?share_source=copy_web&vd_source=58144939e34acbbb38e3f36c750e8498)

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=452124160&bvid=BV1Wj411s7hJ&cid=1361902160&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>