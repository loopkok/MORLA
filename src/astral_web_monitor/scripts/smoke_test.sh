#!/bin/bash
# astral_web_monitor 快速自检脚本
# 用法: ./smoke_test.sh
# 前提: monitor 已在 8080 端口运行, driver 以 dry_run 模式运行

set -e
HOST="http://localhost:8080"
PASS=0; FAIL=0

check() {
  local desc="$1"; local expected="$2"; local actual="$3"
  if echo "$actual" | grep -q "$expected"; then
    echo "  ✅ $desc"
    PASS=$((PASS+1))
  else
    echo "  ❌ $desc (期望含 '$expected', 实际: $actual)"
    FAIL=$((FAIL+1))
  fi
}

echo "=== 1. Health ==="
R=$(curl -s $HOST/api/v1/health)
check "ros_ok=true" '"ros_ok":true' "$R"
check "launch_state 存在" 'launch_state' "$R"

echo "=== 2. Presets ==="
R=$(curl -s $HOST/api/v1/presets)
check "返回预设列表" '"name"' "$R"

echo "=== 3. State ==="
R=$(curl -s $HOST/api/v1/state)
check "ui_state 帧" '"ui_state"' "$R"
check "joints 存在" '"joints"' "$R"
check "health 存在" '"health"' "$R"

echo "=== 4. 未知预设 404 ==="
R=$(curl -s -o /dev/null -w "%{http_code}" -X POST $HOST/api/v1/start -H 'Content-Type: application/json' -d '{"preset":"不存在"}')
check "返回 404" "404" "$R"

echo "=== 5. 无运行时暂停 409 ==="
R=$(curl -s -o /dev/null -w "%{http_code}" -X POST $HOST/api/v1/pause)
check "返回 409" "409" "$R"

echo "=== 5b. 段间回位端点存在（遥操停止时应 409 而非 404）==="
R=$(curl -s -o /dev/null -w "%{http_code}" -X POST $HOST/api/v1/teleop/workpos)
check "工作位 /workpos 返回 409" "409" "$R"
R=$(curl -s -o /dev/null -w "%{http_code}" -X POST $HOST/api/v1/teleop/workpos/direct)
check "段间回位 /workpos/direct 返回 409" "409" "$R"

echo "=== 6. Driver 服务（需 dry_run driver 运行中）==="
for svc in ready home damping position estop; do
  R=$(curl -s -X POST $HOST/api/v1/robot/$svc)
  if echo "$R" | grep -q '"ok":true'; then
    echo "  ✅ /robot/$svc"
    PASS=$((PASS+1))
  elif echo "$R" | grep -q 'detail'; then
    echo "  ⚠️  /robot/$svc (driver 未启动? 跳过)"
  else
    echo "  ❌ /robot/$svc ($R)"
    FAIL=$((FAIL+1))
  fi
done

echo ""
echo "结果: $PASS 通过, $FAIL 失败"
