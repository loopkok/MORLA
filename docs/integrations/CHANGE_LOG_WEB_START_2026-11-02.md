# Web 无参数启动与六场景手册实施日志

日期：2026-11-02，Asia/Shanghai。仓库：loopkok/MORLA，分支：main。本次归档前两轮尚未提交的启动脚本、使用手册与文档修改，并先同步远端修复。

## 远端同步

- 本地起点：`27f2bbcd838369155c8f0d8c272114f7d02ed33f`。
- 抓取到远端：`661de697edb56d0485ba9a1c5124b2923d49e09b`，主题 `fix(cr5): use a valid maintainer email so cr5_mujoco_sim is a ROS package`。
- 远端只修改 `src/robots/dobot/cr5_mujoco_sim/package.xml`，将维护者邮箱由 `robot@localhost` 改为 `maintainers@morla.dev`。这是远端已有修复，不作为本轮新实现重复署名。
- 已核对远端修改与本地启动脚本、CHANGELOG、README 和新增手册没有文件重叠；执行 `git merge --ff-only origin/main` 成功快进，本地未提交内容完整保留，无冲突、无额外合并提交。
- 本轮提交以 `661de69` 为父提交，不回退或覆盖该 ROS 包修复。fetch/merge 使用 GitHub SSH 命令级 URL 重写，未修改持久 remote 配置。

## 启动脚本

修改 `tools/start_morla_web.sh`，保留可执行权限和现有完整控制台入口 `morla_web_manager`，没有改成需要 assembly/deployment 文件的 session CLI。

- 直接运行即可加载 `/opt/ros/humble/setup.bash` 和本仓库 `install/setup.bash`，启动 Web 后由页面配置工作站；机器人和输入采集仍由操作员在页面启动。
- 将“组合配置”“激活运动许可”等旧终端提示更新为“工作站”“开始遥操”，输出当前配置、依赖检查、保存、启动、连接输入和准备设备的实际顺序。
- 说明 Vader M3、Quest 右 grip、Web 和键盘 S 的启动方式，裸手需要 Web/S；本地 viewer 在服务主机桌面打开。
- 校验 PORT 为 1–65535、最多五位十进制数字；有效前导零归一化。保留默认 8080、ROS_DOMAIN_ID 默认 91、ROS_LOCALHOST_ONLY=1 和 OPEN_BROWSER/PORT 覆盖入口。
- 检查 curl、ROS 与工作区 setup、ros2 和安装后的可执行入口，缺失时输出可操作的错误信息。
- 复用服务前检查 `/api/v2/components`，不再仅因根页面返回成功就认定 MORLA 已启动。根页面可访问而公共 API 不可用时报告冲突，提示检查进程或换端口；不自动终止其他服务。
- curl 本机探测设置连接/总超时并绕过本机代理；缺少 xdg-open 时输出手动打开地址。
- 已有服务只复用，不会因脚本更新自动加载新代码，提示先停止旧服务再启动。退出提示改为先停止工作站再 Ctrl+C。

使用：

```bash
cd /实际路径/MORLA
bash tools/start_morla_web.sh
# 不自动打开浏览器
OPEN_BROWSER=0 bash tools/start_morla_web.sh
# 改用其他端口
PORT=9090 bash tools/start_morla_web.sh
```

## 从零 Web 操作手册

新增 [六场景手册](WEB_TELEOP_SIX_SCENARIOS_2026-11-02.md)，同步 Web manager README 链接。手册覆盖无用户预写配置文件时的构建、依赖检查、空白 Web 启动、Vader hidraw 身份/权限、Quest ADB/TCP、页面公共流程和六种配置：

- Vader → Nero 双臂。
- Vader → CR5。
- Quest 双手柄 → Nero 双臂。
- Quest 裸双手 → Nero 双臂 + 双 XHand。
- Quest 右手柄 → CR5。
- Quest 裸右手 → CR5。

各场景说明输入真实后端与执行器仿真后端的区分、映射端口/控制方式、双 XHand 的 flange 安装、持续跟随和按住使能差别、网页/本地/同时显示、保存恢复和停止顺序。

明确保留的限制：一只 Vader 的同一组六轴映射双臂时两臂联动，没有新增左右臂切换功能；裸手捏合不自动启动全局遥操；本地窗口不会在远程浏览器电脑弹出。Linux HID/ROS 流程不能原样当作 macOS 实体输入流程。新增手册是说明文档，不是另外生成六份需要用户维护的配置文件。

## 本轮同步后验证

- `bash -n tools/start_morla_web.sh` 通过。
- 9 项脚本分支冒烟检查通过：临时本地 HTTP 服务的 MORLA 公共 API 复用、仅普通根页面服务的拒绝、6 个非法端口（0、65536、-1、abc、999999、1.5）拒绝，以及当前无 ROS 安装环境的明确报错。HTTP 服务仅用于检查启动前分支，结束后已关闭。
- 远端同步后的 CR5 package.xml 用 Python XML 解析成功，并确认维护者邮箱为 `maintainers@morla.dev`。这不是在 macOS 上完成 ROS/colcon 包识别验收。
- `git diff --check` 通过；提交前再次检查暂存区空白错误及脚本语法。
- 六种等价装配的编译/MuJoCo 模型加载检查在编写手册时完成：Nero 双臂 14 维、Nero 双臂双手 38 维、CR5 6 维。本次仅同步邮箱和提交文档/脚本，未重复运行整套机器人测试，也不把上一轮 322 项 Python 测试计作本轮新运行。

当前机器缺少 Linux ROS 2 Humble，未执行本轮完整 Web 子进程启动、colcon、实体 Vader/Quest、真实机器人或图形 viewer 验收；没有改动运动控制算法、前端源码或已构建 web/dist，因此没有重新构建前端。远端 CI 状态不由本地检查推断。

## 提交与推送记录

本轮先完成日志与本地提交，再以普通快进方式推送 origin/main；实际提交 SHA、推送返回值与远端核对结果另记推送回执。不会使用 force push。
