"""Startup inspection: never kill a live station unless the user asked --replace."""

from __future__ import annotations

from dataclasses import dataclass

from robot_station.config import StationConfig
from robot_station.lock import MOTION_LOCK, lock_held, port_open


@dataclass(frozen=True)
class RuntimeStatus:
    http_up: bool
    motion_up: bool
    lock_held: bool


@dataclass(frozen=True)
class StartupPlan:
    action: str
    reason: str


def inspect_runtime(cfg: StationConfig) -> RuntimeStatus:
    return RuntimeStatus(
        http_up=port_open(cfg.http_port, "127.0.0.1"),
        motion_up=port_open(cfg.motion_port, cfg.motion_host or "127.0.0.1"),
        lock_held=lock_held(MOTION_LOCK),
    )


def plan_startup(
    status: RuntimeStatus,
    *,
    replace: bool = False,
    web_only: bool = False,
    motion_only: bool = False,
    in_process: bool = False,
) -> StartupPlan:
    if motion_only:
        if status.lock_held and not replace:
            return StartupPlan(
                "refuse",
                "运动锁被占用：站控已经在跑或正在启动。确认没人用再加 --replace。",
            )
        if status.motion_up and not replace:
            return StartupPlan("refuse", "运动端口已被占用。确认没人用再加 --replace。")
        if replace and (status.lock_held or status.motion_up):
            return StartupPlan("replace_then_start", "按 --replace 结束后再拉运动进程")
        return StartupPlan("start", "拉起运动进程")

    if web_only:
        if not status.motion_up:
            return StartupPlan("refuse", "网页单独启动需要运动进程已在 9470 听着。请先 python run_station.py")
        return StartupPlan("web_only", "只跑网页，接到已有运动进程")

    if in_process:
        if status.lock_held and not replace:
            return StartupPlan("refuse", "运动锁被占用。确认没人用再加 --replace。")
        if replace and (status.http_up or status.motion_up or status.lock_held):
            return StartupPlan("replace_then_start", "按 --replace 结束后再单进程启动")
        return StartupPlan("start", "单进程启动")

    if replace:
        if status.http_up or status.motion_up or status.lock_held:
            return StartupPlan("replace_then_start", "按 --replace 结束旧站控再启动")
        return StartupPlan("start", "正常启动")

    if status.http_up and status.motion_up:
        return StartupPlan("attach", "站控已在运行，复用，不杀进程")

    if status.motion_up and not status.http_up:
        return StartupPlan("web_only", "运动进程还在，只补网页")

    if status.http_up and not status.motion_up:
        return StartupPlan(
            "refuse",
            "9400 已被占用但运动进程不在。不要硬抢。确认没人用后：python run_station.py --replace",
        )

    if status.lock_held:
        return StartupPlan(
            "refuse",
            "运动锁被占用但端口还没起来，可能上一份正在启动。等几秒；若确认没人用再加 --replace。",
        )

    return StartupPlan("start", "正常启动")


def should_open_browser(cfg: StationConfig, status: RuntimeStatus | None = None) -> bool:
    return bool(getattr(cfg, "open_browser", True))
