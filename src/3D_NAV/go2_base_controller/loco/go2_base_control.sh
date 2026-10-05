#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMERGENCY_STOP_SCRIPT="${SCRIPT_DIR}/emergency_stop_monitor.py"
GO2_CONTROLLER_SCRIPT="${SCRIPT_DIR}/go2_base_controller.py"

GO2_IFACE="${GO2_IFACE:-enp86s0}"
GO2_PEER="${GO2_PEER:-192.168.123.161}"
GO2_DOMAIN_ID="${GO2_DOMAIN_ID:-0}"
GO2_DDS_TRACE="${GO2_DDS_TRACE:-/tmp/cdds.LOG}"
GO2_LOG_DIR="${GO2_LOG_DIR:-/tmp}"

# PID文件
EMERGENCY_PID="/tmp/go2_emergency_stop.pid"
GO2_CONTROLLER_PID="/tmp/go2_controller.pid"
EMERGENCY_LOG="${GO2_LOG_DIR}/go2_emergency_stop.log"
GO2_CONTROLLER_LOG="${GO2_LOG_DIR}/go2_controller.log"

pid_is_running() {
    local pid="$1"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

stop_process_from_pidfile() {
    local pidfile="$1"
    local label="$2"
    if [ -f "$pidfile" ]; then
        local pid
        pid=$(cat "$pidfile")
        if pid_is_running "$pid"; then
            echo "停止${label} (PID: $pid)"
            kill "$pid"
        fi
        rm -f "$pidfile"
    fi
}

route_dev_for_peer() {
    local peer="$1"
    ip route get "$peer" 2>/dev/null | awk '{
        for (i = 1; i <= NF; ++i) {
            if ($i == "dev") {
                print $(i + 1)
                exit
            }
        }
    }'
}

pick_iface_source_ip() {
    local iface="$1"
    local peer="$2"
    local peer_prefix
    local first_ip=""

    peer_prefix=$(echo "$peer" | awk -F. '{print $1"."$2"."$3}')

    while read -r cidr; do
        local ip
        local ip_prefix
        ip="${cidr%/*}"
        [ -n "$first_ip" ] || first_ip="$ip"
        ip_prefix=$(echo "$ip" | awk -F. '{print $1"."$2"."$3}')
        if [ "$ip_prefix" = "$peer_prefix" ]; then
            echo "$ip"
            return 0
        fi
    done < <(ip -4 -o addr show dev "$iface" scope global | awk '{print $4}')

    [ -n "$first_ip" ] && echo "$first_ip"
}

ensure_peer_route() {
    if [ -z "$GO2_PEER" ]; then
        return 0
    fi

    if ! ip link show "$GO2_IFACE" >/dev/null 2>&1; then
        echo "错误: 网卡 ${GO2_IFACE} 不存在"
        return 1
    fi

    local current_dev
    current_dev=$(route_dev_for_peer "$GO2_PEER")
    if [ "$current_dev" = "$GO2_IFACE" ]; then
        echo "DDS路由已正确指向 ${GO2_IFACE} -> ${GO2_PEER}"
        return 0
    fi

    local src_ip
    src_ip=$(pick_iface_source_ip "$GO2_IFACE" "$GO2_PEER")

    echo "修正DDS路由: ${GO2_PEER} 当前走 ${current_dev:-unknown}，切换到 ${GO2_IFACE}"
    local route_cmd=(ip route replace "$GO2_PEER" dev "$GO2_IFACE" metric 50)
    if [ -n "$src_ip" ]; then
        route_cmd+=(src "$src_ip")
    fi

    if [ "$EUID" -eq 0 ]; then
        "${route_cmd[@]}"
    else
        sudo "${route_cmd[@]}"
    fi

    current_dev=$(route_dev_for_peer "$GO2_PEER")
    if [ "$current_dev" != "$GO2_IFACE" ]; then
        echo "错误: DDS路由修正失败，${GO2_PEER} 仍未走 ${GO2_IFACE}"
        return 1
    fi

    echo "DDS路由修正完成: ${GO2_PEER} -> ${GO2_IFACE}${src_ip:+ (src ${src_ip})}"
}

build_common_args() {
    COMMON_ARGS=(--iface "$GO2_IFACE" --domain-id "$GO2_DOMAIN_ID")
    if [ -n "$GO2_PEER" ]; then
        COMMON_ARGS+=(--peer "$GO2_PEER")
    fi
    if [ -n "$GO2_DDS_TRACE" ]; then
        COMMON_ARGS+=(--dds-trace "$GO2_DDS_TRACE")
    fi
}

start() {
    echo "启动Go2控制系统..."
    echo "DDS配置: iface=${GO2_IFACE}, peer=${GO2_PEER:-none}, domain=${GO2_DOMAIN_ID}"
    stop_process_from_pidfile "$GO2_CONTROLLER_PID" "Go2控制器"
    stop_process_from_pidfile "$EMERGENCY_PID" "急停监控器"
    ensure_peer_route || return 1
    build_common_args
    
    # 启动急停监控器
    echo "启动急停监控器..."
    python3 "$EMERGENCY_STOP_SCRIPT" "${COMMON_ARGS[@]}" > "$EMERGENCY_LOG" 2>&1 &
    echo $! > "$EMERGENCY_PID"
    echo "急停监控器启动 (PID: $(cat $EMERGENCY_PID))"
    echo "急停监控日志: $EMERGENCY_LOG"
    
    # 等待2秒再启动控制器
    sleep 2
    
    # 启动Go2控制器
    echo "启动Go2控制器..."
    python3 "$GO2_CONTROLLER_SCRIPT" "${COMMON_ARGS[@]}" > "$GO2_CONTROLLER_LOG" 2>&1 &
    echo $! > "$GO2_CONTROLLER_PID"
    echo "Go2控制器启动 (PID: $(cat $GO2_CONTROLLER_PID))"
    echo "Go2控制器日志: $GO2_CONTROLLER_LOG"
    
    echo "Go2控制系统启动完成！"
}

stop() {
    echo "停止Go2控制系统..."
    stop_process_from_pidfile "$GO2_CONTROLLER_PID" "Go2控制器"
    stop_process_from_pidfile "$EMERGENCY_PID" "急停监控器"
    echo "Go2控制系统已停止！"
}

case "$1" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        sleep 2
        start
        ;;
    *)
        echo "用法: $0 {start|stop|restart}"
        exit 1
        ;;
esac 
