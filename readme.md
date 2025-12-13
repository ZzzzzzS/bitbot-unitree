@mainpage

# BitbotEncos

Bitbot Unitree是Bitbot机器人控制框架的一个实例。该实例提供[Unitree SDK](https://github.com/unitreerobotics/unitree_sdk2)的bitbot封装，使用户可以快速在[unitree g1](https://www.unitree.com/g1/)机器人实物上部署自己的控制算法（仅unitree开发版机器人支持）。

> 该Bitbot实例基于Bitbot机器人软件框架设计，关于Bitbot框架的更多详细信息请参阅其[官方网站](https://bitbot.lmy.name/)。

> 该Bitbot对unitree sdk2进行封装，关于unitree sdk的更多信息可参考其[官方文档](https://support.unitree.com/home/en/G1_developer/about_G1)

# 软件配置

## 安装Bitbot Unitree程序

Bitbot Unitree需要在具备实时内核的linux操作系统上运行，已经在unitree g1内部电脑上测试。Bitbot Unitree仓库包含电机驱动，状态机，数据记录和**强化学习部署推理**等功能，其中**强化学习部署推理**功能由子仓库[CtrlZ](https://github.com/ZzzzzzS/CtrlZ)提供。Bitbot Unitree仅需依赖ONNXRuntime，用户可参阅[ONNXRuntime官方文档](https://github.com/microsoft/onnxruntime)来安装**c++版本**的依赖。

安装完成后使用git递归克隆来下载本仓库。

```bash
git clone https://github.com/ZzzzzzS/bitbot-unitree.git --recursive
```

> [CtrlZ](https://github.com/ZzzzzzS/CtrlZ)是一个硬件无关的多线程强化学习部署框架，用于简化学习类机器人运动控制算法在实际机器人上的部署，提升部署的灵活性，通用性，简化部署流程，同时利用多线程推理加速来提升实时性。详细信息可参阅[CtrlZ文档](https://opensource.zzshub.cn/CtrlZ/index.html)。

> [ONNXRuntime](https://onnxruntime.ai/)是一个通用神经网络推理框架，可以根据不同硬件设备选择不同后端来推理，在unitree g1上可以从源码编译tensorRT后端来实现极致的加速效果。但针对一般小模型，通用cpu版本也不会导致计算超时。

## 安装Bitbot Copilot机器人远程控制客户端

Bitbot Unitree将会在机器人上部署运行，并通过ssh连接进行远程开发与调试。用户可以在本地计算机上安装[Bitbot Copilot](https://github.com/ZzzzzzS/BitbotCopilot)来远程控制机器人。Bitbot Copilot预编译版本可以从[这里下载](https://github.com/ZzzzzzS/BitbotCopilot/releases/tag/v1.0)。

> Bitbot Copilot是一个Bitbot机器人控制框架的图形化前端程序。该程序支持使用键盘，手柄或触摸板来实时控制使用Bitbot后端的机器人，并支持实时数据绘制，历史数据查看等功能，以及远端机器人数据管理，自动初始化等高级功能。
> ![bitbot copilot](https://github.com/ZzzzzzS/BitbotCopilot/raw/main/doc/MainPage.png)

# 设备

* UnitreeJoint，用于控制电机
* UnitreeImu，用于读取IMU数据
* UnitreeGamepad，用户读取unitree遥控器输入，该类型会自动触发[Bitbot Event](https://bitbot.lmy.name/docs/bitbot-programming#event-%E4%BA%8B%E4%BB%B6)

# 部署

## 准备阶段

Bitbot Unitree支持一键切换mujoco仿真器和实物机器人部署，通过``CMakeLists.txt``中的``BUILD_SIMULATION``宏来定义使用仿真环境或实物机器人，mujoco仿真环境支持在windows/linux下使用，实物机器人只能在实时linux系统下部署，推荐使用unitree机器人内部计算机部署。* 在使用mujoco仿真时，可能会自动下载一些依赖项，请确保网络畅通。

> 在使用实物机器人部署时，机器人会自动切换至unitree开发者模式，无需使用unitree遥控器切换。

## 运行阶段

**机器人运行时请务必注意安全!**
机器人程序运行后可以使用[Bitbot Copilot](https://github.com/ZzzzzzS/BitbotCopilot)进行控制。连接到机器人后依次点击键盘``9``，``8``，``p``按钮来完成机器人上电，运行，复位功能，在机器人复位完成后点击键盘``r``按钮启动机器人平衡控制，演示结束时点击键盘``空格键``退出程序。*该流程用户可自定义，这里展示的是默认的操作流程*。
> Bitbot Copilot支持自动化启动，简化按键流程，详细信息请参阅其开源仓库。

# 免责声明

机器人实物实验是具备一定风险等级的实验。机器人运行时，请勿将身体任何部位伸进机器人的任何空洞内或靠近关节附近。
**开机后请务必确保紧握急停按钮。建议用户在实物实验前进行充分的仿真验证，在实物实验时先不上电空跑来确保安全！** 因不当使用本程序造成的一切后果与作者无关。

# 致谢

小伙伴们，感谢你们关注这个开源项目！开源的魅力在于汇聚众人的智慧与力量，共同打造更优秀的软件。如果你在使用过程中遇到问题，无论是功能上的困惑、操作上的卡顿，还是发现了潜在的漏洞，都请大胆地提issue。你的每一个反馈都是我们进步的阶梯，我们期待与你携手，让这个项目不断完善。
同时，我们也热忱欢迎每一位愿意贡献的伙伴。无论是代码优化、功能拓展，还是文档完善，漏洞修复，你的每一份付出都将为项目添砖加瓦。众人拾柴火焰高，让我们一起努力，让Bitbot在大家的共同努力下，绽放出更耀眼的光芒！Make Bitbot Everywhere!
