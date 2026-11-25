#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: %s [--bma400] [TOUCH_LAUNCHER]\n' "${0##*/}"
}

with_bma400=false
case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    --bma400) with_bma400=true; shift ;;
esac
if (( $# > 1 )) || [[ "${1:-}" == -* ]]; then
    usage >&2
    exit 2
fi

# Select producer ports; keep the two recording streams independent.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
touch_launcher="${1:-$PROJECT_DIR/run_touch.sh}"
touch_source="${RIGHT_COMPENSATED_SOURCE:-/icub/skin/right_hand_comp}"
sources=("$touch_source")
stream_names=(touch)
if "$with_bma400"; then
    if [[ -z "${BMA400_YARP_PORT:-}" ]]; then
        BMA400_YARP_PORT="$(
            source "$PROJECT_DIR/config/bma400.env"
            printf '%s' "$BMA400_YARP_PORT"
        )"
    fi
    sources+=("$BMA400_YARP_PORT")
    stream_names+=(bma400)
fi

for command in yarp yarpdatadumper timeout; do
    command -v "$command" >/dev/null || { echo "Missing command: $command" >&2; exit 1; }
done

launcher_pids=()
dumper_pids=()
dumper_ports=()
connected=0

port_exists() {
    timeout 2 yarp exists "$1" >/dev/null 2>&1
}

wait_for_port() {
    local port="$1" pid="$2"
    local deadline=$((SECONDS + $3))
    while (( SECONDS < deadline )) && kill -0 "$pid" 2>/dev/null; do
        if port_exists "$port"; then
            return 0
        fi
        sleep 0.2
    done
    printf 'Failed to open %s (process %s). Check its output above.\n' "$port" "$pid" >&2
    return 1
}

stop_dumpers() {
    local ok=0 i pid attempt
    for ((i = 0; i < connected; ++i)); do
        timeout 3 yarp disconnect "${sources[i]}" "${dumper_ports[i]}" >/dev/null 2>&1 || ok=1
    done
    connected=0

    for i in "${!dumper_pids[@]}"; do
        pid="${dumper_pids[i]}"
        if ! kill -0 "$pid" 2>/dev/null ||
           ! printf 'quit\n' | timeout 3 yarp rpc "${dumper_ports[i]}/rpc" >/dev/null 2>&1; then
            printf 'Could not stop the %s dumper cleanly.\n' "${stream_names[i]}" >&2
            kill -TERM "$pid" 2>/dev/null || true
            ok=1
        fi
    done
    for i in "${!dumper_pids[@]}"; do
        pid="${dumper_pids[i]}"
        for ((attempt = 0; attempt < 50; ++attempt)); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.2
        done
        if kill -0 "$pid" 2>/dev/null; then
            printf '%s dumper did not exit; its log may be incomplete.\n' "${stream_names[i]}" >&2
            kill -KILL "$pid" 2>/dev/null || true
            ok=1
        fi
        wait "$pid" || ok=1
    done
    dumper_pids=()
    return "$ok"
}

cleanup() {
    stop_dumpers || true
    if (( ${#launcher_pids[@]} )); then
        kill -TERM "${launcher_pids[@]}" 2>/dev/null || true
        wait "${launcher_pids[@]}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Start only missing producers. Existing acquisition processes are not owned
# by this recording session and must not be stopped by its cleanup trap.
if ! port_exists "$touch_source"; then
    bash "$touch_launcher" &
    launcher_pids+=("$!")
    wait_for_port "$touch_source" "$!" 75
fi
if "$with_bma400" && ! port_exists "${sources[1]}"; then
    bash "$PROJECT_DIR/run_bma400.sh" --yarp-port "${sources[1]}" &
    launcher_pids+=("$!")
    wait_for_port "${sources[1]}" "$!" 20
fi

while read -r -p "Type yes to start recording: " answer; do
    [[ "$answer" == yes ]] && break
done
[[ "${answer:-}" == yes ]] || exit 0

output_root="$PROJECT_DIR/data"
mkdir -p "$output_root"
session="$(mktemp -d "$output_root/touch_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
cd "$session"
echo "Session directory: $session"

# Create one YARP dumper per stream. Receiver times share the host clock;
# they do not guarantee hardware-synchronous sampling.
for i in "${!sources[@]}"; do
    port="/primi-ball-balance/record/${HOSTNAME:-localhost}/${session##*/}/${stream_names[i]}"
    dumper_ports+=("$port")
    # Both dumpers timestamp reception using the recording PC's system clock.
    env -u YARP_CLOCK yarpdatadumper --name "$port" --type bottle \
        --dir "${stream_names[i]}" --txTime --rxTime &
    dumper_pids+=("$!")
done

for i in "${!sources[@]}"; do
    wait_for_port "${dumper_ports[i]}" "${dumper_pids[i]}" 10
    wait_for_port "${dumper_ports[i]}/rpc" "${dumper_pids[i]}" 10
done
for i in "${!sources[@]}"; do
    if ! timeout 5 yarp connect "${sources[i]}" "${dumper_ports[i]}" tcp; then
        printf 'Could not connect %s. Recording incomplete. Check %s\n' "${sources[i]}" "$session" >&2
        exit 1
    fi
    connected=$((connected + 1))
done

echo "Recording ${stream_names[*]}."
while read -r -p "Type stop to finish: " answer; do
    [[ "$answer" == stop ]] && break
done

result=0
stop_dumpers || result=1
for stream in "${stream_names[@]}"; do
    if [[ ! -s "$stream/data.log" ]]; then
        printf 'No samples saved for %s.\n' "$stream" >&2
        result=1
    fi
done
if (( result )); then
    printf 'Recording incomplete. Check %s\n' "$session" >&2
else
    printf 'Recording saved in %s\n' "$session"
fi
exit "$result"
