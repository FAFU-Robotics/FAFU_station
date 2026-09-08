from __future__ import annotations

import asyncio
import json
import logging
import mimetypes
import os
import secrets
import struct
import subprocess
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

from websockets.datastructures import Headers
from websockets.http11 import Request, Response

try:
    # websockets 13+（conda 里常见）
    from websockets.asyncio.server import ServerConnection, serve

    _WS_LEGACY = False
except ImportError:
    # Desktop/venv 目前是 12.0，没有 websockets.asyncio；12 也只认 HTTP GET
    from websockets.legacy.server import WebSocketServerProtocol as ServerConnection
    from websockets.server import serve

    _WS_LEGACY = True

from robot_station.config import StationConfig
from robot_station.service import Station
from robot_station.telem import pack_telem

logger = logging.getLogger("station.web")

STATIC = Path(__file__).resolve().parent / "static"


def _teleop_hot(st: Station) -> bool:
    fn = getattr(st, "teleop_hot", None)
    return bool(fn()) if callable(fn) else False
VID_HDR = struct.Struct("!BBHHHI")
REASONS = {
    200: "OK",
    204: "No Content",
    400: "Bad Request",
    401: "Unauthorized",
    404: "Not Found",
    405: "Method Not Allowed",
}


def _http(status: int, body: bytes, content_type: str, extra: list[tuple[str, str]] | None = None) -> Response:
    headers = Headers()
    headers["Content-Type"] = content_type
    headers["Content-Length"] = str(len(body))
    headers["Cache-Control"] = "no-store"
    headers["Connection"] = "close"
    if extra:
        for key, value in extra:
            headers[key] = value
    return Response(status, REASONS.get(status, "OK"), headers, body)


def _json(status: int, payload: dict, extra: list[tuple[str, str]] | None = None) -> Response:
    return _http(status, json.dumps(payload).encode("utf-8"), "application/json; charset=utf-8", extra)


def _open_browser(url: str, wait: bool = False) -> None:
    def _go() -> None:
        time.sleep(0.5)
        if os.environ.get("STATION_NO_BROWSER"):
            return
        try:
            from station_desktop import _app_shell_argv

            argv = _app_shell_argv(url)
            if argv:
                subprocess.Popen(argv)
                logger.info("已打开本机窗口")
                return
        except Exception:
            pass
        if os.name != "nt":
            try:
                subprocess.Popen(
                    ["xdg-open", url],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                logger.info("已尝试打开本机浏览器")
                return
            except Exception:
                pass
        try:
            import webbrowser

            webbrowser.open(url)
            logger.info("已尝试打开本机浏览器")
        except Exception:
            logger.warning("没能自动打开浏览器，请手动访问 %s", url)

    thread = threading.Thread(target=_go, name="open-browser", daemon=not wait)
    thread.start()
    if wait:
        thread.join(timeout=4.0)


def _ws_path(ws: ServerConnection) -> str:
    req = getattr(ws, "request", None)
    if req is not None:
        return urlsplit(req.path).path
    return urlsplit(getattr(ws, "path", "/") or "/").path


def serve_http(station: Station, cfg: StationConfig, stop_event: threading.Event) -> None:
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def dispatch(path: str, method: str, headers: Headers) -> Response | None:
        path = urlsplit(path).path
        method = method.upper()
        upgrade = (headers.get("Upgrade") or "").lower()

        if upgrade == "websocket":
            if path in ("/ws", "/ws/cmd", "/ws/video", "/ws/rtt"):
                return None
            return _http(404, b"not found", "text/plain; charset=utf-8")

        if path in ("/", "/login", "/index.html") and method == "GET":
            return _file(STATIC / "index.html")
        if path in ("/lab", "/lab.html") and method == "GET":
            return _file(STATIC / "lab.html")
        if path == "/api/lab" and method == "GET":
            from robot_station.lock import list_ipv4_addrs
            from robot_station.runtime import inspect_runtime

            st = inspect_runtime(cfg)
            return _json(
                200,
                {
                    "ok": True,
                    "station_port": cfg.http_port,
                    "motion_up": bool(st.motion_up),
                    "http_up": True,
                    "host_ip": station.host_ip,
                    "ips": list_ipv4_addrs() if cfg.listen_host in ("0.0.0.0", "::") else [station.host_ip],
                },
            )
        if path.startswith("/static/") and method == "GET":
            rel = path[len("/static/") :]
            target = (STATIC / rel).resolve()
            if not str(target).startswith(str(STATIC.resolve())) or not target.is_file():
                return _http(404, b"not found", "text/plain; charset=utf-8")
            return _file(target)
        if path == "/api/info" and method == "GET":
            snap = station.snapshot()
            arm = (snap.get("arm") or {}).get("backend") or cfg.arm
            from robot_station.runtime import station_code_rev
            from robot_station.serial_guard import live_serial_allowed

            live = live_serial_allowed() or arm == "fafu"
            return _json(
                200,
                {
                    "ok": True,
                    "title": "FAFU 机械臂站控",
                    "robot_id": cfg.robot_id,
                    "host_ip": station.host_ip,
                    "http_port": cfg.http_port,
                    "control_hz": cfg.control_hz,
                    "video_hz": cfg.video_hz,
                    "arm": arm,
                    "camera": cfg.camera,
                    "live_serial_allowed": live,
                    "arm_allow_motion": bool(cfg.arm_allow_motion) or arm == "fafu",
                    "arm_port": cfg.arm_port or "cfg/auto",
                    "code_rev": station_code_rev(),
                },
            )
        if path == "/api/scripts" and method == "GET":
            from robot_station.adapters.fafu_arm import list_sdk_scripts, map_script

            names = list_sdk_scripts(cfg.arm_sdk or None)
            return _json(
                200,
                {
                    "ok": True,
                    "scripts": [
                        {"name": n, "mapped": map_script(n) is not None} for n in names
                    ],
                },
            )
        if path.split("?", 1)[0] == "/api/traj" and method == "GET":
            from robot_station.traj import list_recordings

            return _json(200, {"ok": True, "files": list_recordings()})
        if path.split("?", 1)[0] == "/api/traj" and method == "DELETE":
            from urllib.parse import parse_qs

            from robot_station.traj import delete_recording

            qs = parse_qs(urlsplit(path).query)
            name = str((qs.get("name") or qs.get("file") or [""])[0] or "")
            try:
                gone = delete_recording(name)
            except FileNotFoundError as exc:
                return _json(404, {"ok": False, "error": str(exc)})
            except OSError as exc:
                return _json(400, {"ok": False, "error": f"删除失败: {exc}"})
            return _json(200, {"ok": True, "name": gone.name})
        return _http(404, b"not found", "text/plain; charset=utf-8")

    def process_request(connection: ServerConnection, request: Request) -> Response | None:
        method = getattr(request, "method", "GET") or "GET"
        return dispatch(request.path, method, request.headers)

    async def process_request_legacy(path: str, request_headers: Headers):
        resp = dispatch(path, "GET", request_headers)
        if resp is None:
            return None
        return resp.status_code, list(resp.headers.raw_items()), resp.body or b""

    async def handler(ws: ServerConnection) -> None:
        path = _ws_path(ws)
        if path == "/ws/video":
            await _video(ws, station, cfg)
        elif path == "/ws/rtt":
            await _rtt(ws)
        elif path == "/ws/cmd":
            await _cmd(ws, station, cfg)
        else:
            await _telem(ws, station, cfg)

    async def runner() -> None:
        async with serve(
            handler,
            cfg.listen_host,
            cfg.http_port,
            process_request=process_request_legacy if _WS_LEGACY else process_request,
            ping_interval=20,
            ping_timeout=20,
            max_size=2**22,
            compression=None,
        ):
            local = f"http://127.0.0.1:{cfg.http_port}"
            lan = f"http://{station.host_ip}:{cfg.http_port}"
            logger.info("已启动。不要关闭这个终端。")
            logger.info("机械臂入口： %s", local)
            if lan != local:
                from robot_station.lock import list_ipv4_addrs

                logger.info("局域网： %s", lan)
                extras = [ip for ip in list_ipv4_addrs() if ip != station.host_ip]
                if extras:
                    logger.info("其它网卡： %s", "  ".join(f"http://{ip}:{cfg.http_port}" for ip in extras))
            if getattr(cfg, "open_browser", True):
                _open_browser(local)
            while not stop_event.is_set():
                await asyncio.sleep(0.2)

    try:
        loop.run_until_complete(runner())
    finally:
        loop.close()


def _file(path: Path) -> Response:
    data = path.read_bytes()
    mime, _ = mimetypes.guess_type(str(path))
    if path.suffix == ".js":
        mime = "text/javascript; charset=utf-8"
    elif path.suffix == ".css":
        mime = "text/css; charset=utf-8"
    elif path.suffix == ".html":
        mime = "text/html; charset=utf-8"
    return _http(200, data, mime or "application/octet-stream")


async def _rtt(ws: ServerConnection) -> None:
    """Empty socket: ping/pong must not sit behind pose or video frames."""
    try:
        async for raw in ws:
            if isinstance(raw, bytes):
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("t") != "ping":
                continue
            t_us = int(msg.get("t_us") or 0)
            now_us = int(time.monotonic() * 1e6) & 0xFFFFFFFF
            await ws.send(json.dumps({"t": "pong", "id": msg.get("id"), "t_us": t_us, "s_us": now_us}))
    except Exception:
        return


async def _cmd(ws: ServerConnection, st: Station, cfg: StationConfig) -> None:
    """Command/ack socket. No 100 Hz snapshots on this connection."""
    cid = secrets.token_hex(4)
    st.add_client(cid)
    try:
        await ws.send(json.dumps({"t": "hello", "cid": cid}))
    except Exception:
        st.drop_client(cid)
        return
    try:
        async for raw in ws:
            if isinstance(raw, bytes):
                continue
            await _on_text(st, cid, raw, ws)
    finally:
        st.drop_client(cid)


async def _telem(ws: ServerConnection, st: Station, cfg: StationConfig) -> None:
    """Binary 100 Hz pose + 10 Hz JSON HUD. Incoming frames ignored."""
    wait_snap = getattr(st, "wait_snap", None)
    fallback = 1.0 / max(1, cfg.telemetry_hz)
    loop = asyncio.get_running_loop()
    box: asyncio.Queue = asyncio.Queue(maxsize=1)
    halt = threading.Event()
    decorate = getattr(st, "_decorate", None)

    def _put(item: tuple[bytes, str | None]) -> None:
        if box.full():
            try:
                box.get_nowait()
            except asyncio.QueueEmpty:
                pass
        box.put_nowait(item)

    def pump() -> None:
        last_seq = -1
        last_hud = 0.0
        while not halt.is_set():
            try:
                if callable(wait_snap):
                    try:
                        seq, data = wait_snap(last_seq, 0.1, False)
                    except TypeError:
                        seq, data = wait_snap(last_seq, 0.1)
                    if seq == last_seq:
                        continue
                    last_seq = seq
                else:
                    time.sleep(fallback)
                    data = st.snapshot()
                    seq = int((data or {}).get("seq") or last_seq + 1)
                    last_seq = seq
                blob = pack_telem(data or {})
                hud = None
                now = time.monotonic()
                arm = (data or {}).get("arm") or {}
                err_hud = bool((data or {}).get("cmd_err") or arm.get("ik_err"))
                if now - last_hud >= 0.1 and (not _teleop_hot(st) or err_hud):
                    payload = decorate(data) if callable(decorate) else data
                    hud = json.dumps({"t": "hud", "d": payload}, ensure_ascii=False, separators=(",", ":"))
                    last_hud = now
                loop.call_soon_threadsafe(_put, (blob, hud))
            except Exception:
                break

    th = threading.Thread(target=pump, name="telem-pump", daemon=True)
    th.start()
    try:
        while True:
            blob, hud = await box.get()
            await ws.send(blob)
            if hud:
                await ws.send(hud)
    except Exception:
        pass
    finally:
        halt.set()


async def _bus(ws: ServerConnection, st: Station, cfg: StationConfig) -> None:
    await _cmd(ws, st, cfg)


async def _on_text(st: Station, cid: str, raw: str, ws: ServerConnection) -> None:
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        return
    kind = msg.get("t")
    if kind == "touch":
        st.on_touch(cid)
        return
    if kind == "park":
        st.on_park(cid)
        return
    if kind == "e":
        st.on_estop("operator")
        return
    if kind == "rtt":
        st.note_rtt_ms(float(msg.get("ms") or 0))
        return
    if kind == "ping":
        t_us = int(msg.get("t_us") or 0)
        now_us = int(time.monotonic() * 1e6) & 0xFFFFFFFF
        await ws.send(json.dumps({"t": "pong", "id": msg.get("id"), "t_us": t_us, "s_us": now_us}))
        return

    async def _rpc(fn, *args):
        return await asyncio.to_thread(fn, *args)

    if kind == "clear":
        err = await _rpc(st.on_clear_estop, cid)
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "clear"}))
    elif kind == "arm_src":
        err = await _rpc(st.on_arm_src, cid, str(msg.get("mode") or ""))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "arm_src"}))
    elif kind == "arm":
        q = msg.get("q") or []
        speed = float(msg.get("speed") or st.cfg.arm_speed_deg_s)
        stream = bool(msg.get("stream"))
        if stream:
            st.on_arm_targets(cid, [float(x) for x in q], speed, True, t0=msg.get("t0"))
            return
        err = await _rpc(st.on_arm_targets, cid, [float(x) for x in q], speed, False)
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "arm"}))
    elif kind == "home":
        speed = float(msg.get("speed") or st.cfg.arm_speed_deg_s)
        err = await _rpc(st.on_home, cid, speed)
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "home"}))
    elif kind == "grip":
        deg = msg.get("deg")
        effort = msg.get("effort")
        open_raw = msg.get("open")
        err = await _rpc(
            st.on_gripper,
            cid,
            None if open_raw is None else bool(open_raw),
            None if deg is None else float(deg),
            None if effort is None else int(effort),
        )
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "grip"}))
    elif kind == "mode":
        err = await _rpc(st.on_mode, cid, str(msg.get("mode") or "Position"))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "mode"}))
    elif kind == "teach":
        q = msg.get("q") or []
        st.on_teach(cid, [float(x) for x in q], t0=msg.get("t0"))
    elif kind == "cart":
        dxyz = [float(x) for x in (msg.get("dxyz") or [0, 0, 0])]
        drpy = [float(x) for x in (msg.get("drpy") or [0, 0, 0])]
        st.on_cartesian(cid, dxyz, drpy, t0=msg.get("t0"), speed=msg.get("speed"))
    elif kind == "cart_go":
        xyz = [float(x) for x in (msg.get("xyz") or [0, 0, 0])]
        rpy = [float(x) for x in (msg.get("rpy") or [0, 0, 0])]
        speed = float(msg.get("speed") or st.cfg.arm_speed_deg_s)
        err = await _rpc(st.on_cartesian_pose, cid, xyz, rpy, speed, msg.get("t0"))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "cart_go"}))
    elif kind == "path":
        raw_wp = msg.get("q") or []
        speed = float(msg.get("speed") or st.cfg.arm_speed_deg_s)
        wps = [[float(x) for x in row] for row in raw_wp]
        dt_raw = msg.get("dt")
        durations = [float(x) for x in dt_raw] if dt_raw else None
        err = await _rpc(st.on_path, cid, wps, speed, durations)
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "path"}))
    elif kind == "rec_start":
        err = await _rpc(st.on_rec_start, cid, msg.get("name"), msg.get("teach"))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "rec_start"}))
    elif kind == "rec_stop":
        err = await _rpc(st.on_rec_stop, cid)
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "rec_stop"}))
    elif kind == "rec_delete":
        err = await _rpc(st.on_rec_delete, cid, str(msg.get("file") or msg.get("path") or ""))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "rec_delete"}))
    elif kind == "replay":
        speed = msg.get("speed")
        err = await _rpc(
            st.on_replay,
            cid,
            str(msg.get("file") or msg.get("path") or ""),
            float(msg.get("rate") or 1.0),
            None if speed is None else float(speed),
        )
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "replay"}))
    elif kind == "link":
        err = await _rpc(st.on_link, cid, str(msg.get("action") or ""))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "link"}))
    elif kind == "script":
        err = await _rpc(st.on_script, cid, str(msg.get("name") or ""))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "script"}))
    elif kind == "power":
        err = await _rpc(st.on_power, cid, bool(msg.get("on", True)))
        await ws.send(json.dumps({"t": "ack", "ok": err is None, "error": err, "op": "power"}))


async def _video(ws: ServerConnection, st: Station, cfg: StationConfig) -> None:
    period = 1.0 / max(1, cfg.video_hz)
    last: list[bytes] = [b""] * 8
    try:
        while True:
            t0 = time.monotonic()
            if not _teleop_hot(st):
                for sid in st.camera.stream_ids():
                    got = st.camera.latest_png(sid)
                    if not got:
                        continue
                    w, h, blob = got
                    if blob == last[sid]:
                        continue
                    last[sid] = blob
                    await ws.send(VID_HDR.pack(0xA1, sid, w, h, 1, len(blob)) + blob)
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, period - elapsed))
    except Exception:
        return
