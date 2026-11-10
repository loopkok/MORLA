# 仿真显示选项缺陷证据（2026-11-10）

对应提交 `deb50e2`（fix(web): make the workstation console panel and simulation options usable）
和记录 [CHANGE_LOG_WORKSTATION_SIM_OPTIONS_2026-11-10.md](../../CHANGE_LOG_WORKSTATION_SIM_OPTIONS_2026-11-10.md)。

## 文件

| 文件 | 内容 |
| --- | --- |
| `web-build.log` | `npm run build`（tsc + vite）输出，生成 `index-BHkflyt9.js` |
| `frontend-tests.log` | `npm run test:workcell`，47 passed / 0 failed |
| `browser.log` | 真实 Chromium 三个场景（无会话、活动会话、结束工作站后）全部通过 |
| `browser-before.log` | 同一脚本跑**修复前**的 `web/dist`（取自提交 `7057691`），“结束后改仿真显示”一项失败，值弹回 `web` |
| `stub-*.log` | 打桩 API 收到的请求；结束后场景没有 `POST /simulation`（修改只写草稿） |
| `stub-api.py` | 隔离打桩 API + 静态服务：`ACTIVE=1` 提供活动会话，`DIST_ROOT` 可指向任意 dist |
| `verify-sim-options.mjs` | Playwright 驱动脚本，三个场景；任一断言失败即非零退出 |

## 复现

```bash
cd docs/integrations/evidence/2026-11-10-sim-options
export BROWSER_EXECUTABLE=<chromium>            # 或修掉脚本里的默认浏览器路径
python3 stub-api.py &                            # 无会话
NODE_PATH=< playwright 所在 node_modules > node verify-sim-options.mjs http://127.0.0.1:18082 no-session

ACTIVE=1 python3 stub-api.py &                   # 活动会话
NODE_PATH=... node verify-sim-options.mjs http://127.0.0.1:18082 active
NODE_PATH=... node verify-sim-options.mjs http://127.0.0.1:18082 stopped

# 修复前对照：取旧 dist 后再跑 stopped 场景
git archive 7057691 src/common/morla_web_manager/web/dist | tar -x -C /tmp/olddist --strip-components=5
DIST_ROOT=/tmp/olddist ACTIVE=1 python3 stub-api.py &
NODE_PATH=... node verify-sim-options.mjs http://127.0.0.1:18082 stopped
```

脚本使用打桩 API，不启动正式 Web 服务，不连接真实设备或 MuJoCo 场景；结论只覆盖
前端状态流，不覆盖真实场景的 `set_options`（本机窗口需要图形环境）。
