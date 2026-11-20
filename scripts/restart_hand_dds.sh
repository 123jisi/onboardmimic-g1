#!/usr/bin/env bash
# Restart only the BrainCo DDS service; startup may open both hands.
set -euo pipefail

unit=brainco_hand.service
check_only=false
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo '用法: bash scripts/restart_hand_dds.sh [--check]'
    echo '--check 只检查服务进程身份，不重启、不发送退出信号。'
    echo '先退出遥操，保持双手空载并清空活动范围；服务重启会默认张开双手。'
    echo '重启后等待最多 30 秒，检查本次服务进程是否识别左右手。'
    exit 0
fi
if [[ "${1:-}" == --check && $# == 1 ]]; then
    check_only=true
elif (( $# )); then
    echo '不支持此参数；使用 --help 查看用法。' >&2
    exit 2
fi

property() { systemctl show "$unit" --property="$1" --value; }
if [[ "$(property LoadState)" != loaded ]]; then
    echo "未找到 $unit，请在 G1 端侧执行。" >&2
    exit 1
fi

old_pid=$(property MainPID)
verify_process() {
    local executable control_group hierarchy controllers group matched=false
    if [[ ! "${old_pid:-0}" =~ ^[0-9]+$ ]] || (( old_pid <= 1 )); then
        echo '服务没有有效的运行进程。' >&2
        return 1
    fi
    # Group=dialout can make /proc/PID/exe unreadable even to User=unitree.
    # cmdline and cgroup remain readable; require both to identify the service.
    if ! IFS= read -r -d '' executable < "/proc/$old_pid/cmdline"; then
        echo '无法读取服务进程启动参数，未发送退出信号。' >&2
        return 1
    fi
    control_group=$(property ControlGroup)
    while IFS=: read -r hierarchy controllers group; do
        if [[ -n "$control_group" && "$group" == "$control_group" && ( -z "$controllers" || "$controllers" == name=systemd ) ]]; then
            matched=true
        fi
    done < "/proc/$old_pid/cgroup"
    if [[ "$executable" != */brainco_hand_server || "$matched" != true || "$(property MainPID)" != "$old_pid" ]]; then
        echo '进程启动参数、控制组或当前服务 PID 不匹配，未发送退出信号。' >&2
        return 1
    fi
}
if [[ "$check_only" == true ]]; then
    verify_process
    echo "检查通过：$unit 的进程身份正确（PID: $old_pid）；未执行重启。"
    exit 0
fi
echo '请确认已退出遥操，双手空载、活动范围清空。重启会默认张开双手。'
echo "正在重启 $unit（原 PID: ${old_pid:-0}）……"
if (( EUID == 0 )); then
    systemctl restart "$unit"
elif [[ "$(property Restart)" == always && "$(property User)" == "$(id -un)" && "${old_pid:-0}" =~ ^[0-9]+$ ]] && (( old_pid > 1 )); then
    # Graceful exit of this user's service; systemd's existing policy restarts it.
    verify_process
    kill -INT "$old_pid"
else
    # Also handles an already-stopped service, where no process can self-restart.
    sudo systemctl restart "$unit"
fi

new_pid=0
logs=''
left_online=false
right_online=false
for (( attempt=0; attempt<30; attempt++ )); do
    new_pid=$(property MainPID)
    if [[ "$new_pid" =~ ^[0-9]+$ ]] && (( new_pid > 1 )) && [[ "$new_pid" != "$old_pid" && "$(property ActiveState)" == active ]]; then
        # Restrict evidence to the new process, never an old successful startup.
        logs=$(journalctl -b -u "$unit" "_PID=$new_pid" --no-pager -o cat 2>/dev/null || true)
        left_online=false
        right_online=false
        [[ "$logs" != *'Starting worker for left'* ]] || left_online=true
        [[ "$logs" != *'Starting worker for right'* ]] || right_online=true
        if [[ "$left_online" == true && "$right_online" == true ]]; then
            printf '%s\n' "$logs"
            echo "成功：左右手均已识别，DDS worker 已启动（新 PID: $new_pid）。"
            echo '请刷新控制页面确认两侧反馈，再按原流程人工进入遥操。'
            exit 0
        fi
    fi
    sleep 1
done

systemctl status "$unit" --no-pager || true
printf '%s\n' "$logs"
if [[ "$left_online" != true ]]; then
    echo '未确认左手上线：本次进程未记录左手 worker 启动，请检查左手供电和连接。' >&2
fi
if [[ "$right_online" != true ]]; then
    echo '未确认右手上线：请检查上面的服务状态与串口探测日志。' >&2
fi
echo '重启／识别验证未通过；未自动进入遥操。' >&2
exit 1
