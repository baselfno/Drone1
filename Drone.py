# -*- coding: utf-8 -*-
# Drone.py — Basel SmartU (Single Drone Version — Offboard NED)

import asyncio
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional, Tuple

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.camera import CameraError


CRUISE_ALTITUDE = 35.0     
CAPTURE_ALTITUDE = 15.0     
PRE_TAKEOFF_HOLD_S = 5
DWELL_SECONDS_PER_POINT = 15
ARRIVAL_RADIUS_M = 5.0
ARRIVAL_TIMEOUT_S = 240.0

LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)



def now_ts() -> str:
    return datetime.utcnow().strftime("%Y-%m-%d_%H-%M-%S")


def log_append(stem: str, line: str):
    fpath = LOG_DIR / f"{stem}.log"
    with open(fpath, "a", encoding="utf-8") as f:
        f.write(f"{now_ts()} | " + line + "\n")


def yaw_from_vector(north: float, east: float) -> float:
    ang = math.degrees(math.atan2(east, north))
    return ang + 360 if ang < 0 else ang


# ======================= بيانات الإبلاغ =======================

@dataclass
class VisitRecord:
    label: str
    type: str
    rel_north: float
    rel_east: float
    photo_flag: bool
    note: str = ""


@dataclass
class DroneSummary:
    drone_name: str
    visited: int = 0
    by_type: Dict[str, int] = field(default_factory=dict)
    photos: int = 0
    records: List[VisitRecord] = field(default_factory=list)

    def add_record(self, rec: VisitRecord):
        self.visited += 1
        self.photos += int(rec.photo_flag)
        self.by_type[rec.type] = self.by_type.get(rec.type, 0) + 1
        self.records.append(rec)

    def pretty(self) -> str:
        lines = [
            f"— MISSION SUMMARY :: {self.drone_name}",
            f"  • Total places visited : {self.visited}",
            f"  • Photos captured      : {self.photos}",
            "  • Types breakdown      : " + (
                ", ".join([f"{k}={v}" for k, v in self.by_type.items()]) or "—"
            ),
        ]
        for rec in self.records:
            lines.append(
                f"    - [{'FLAG=True' if rec.photo_flag else 'FLAG=False'}] "
                f"{rec.label} ({rec.type}) @ N={rec.rel_north:.1f}m, E={rec.rel_east:.1f}m {rec.note}"
            )
        return "\n".join(lines)




LAMPS = [
    ("lamp", "لمبة 1", 207.0, 155.0),
    ("lamp", "لمبة 2", 19.7158, 105.86),
    ("lamp", "لمبة 3", 5.31717, 104.462),
    ("lamp", "لمبة 4", 18.7637, 122.364),
    ("lamp", "لمبة 5", 524.973, -141.123),
    ("lamp", "لمبة 6", 411.443, -132.508),
    ("lamp", "لمبة 7", 349.778, -129.297),
    ("lamp", "لمبة 8", 288.595, -138.744),
]

CHARGERS = [
    ("charger", "محطة شحن 1", 207.0, 155.0),
    ("charger", "محطة شحن 2", -132.232, 99.5319),
    ("charger", "محطة شحن 3", -108.298, 111.632),
    ("charger", "محطة شحن 4", -96.8287, 89.9228),
]

PANELS = [
    ("panel", "اللوح 1", -50.85, 203.759),
    ("panel", "اللوح 2", -47.2063, 201.24),
    ("panel", "اللوح 3", -43.2334, 205.607),
    ("panel", "اللوح 4", -39.3281, 203.148),
    ("panel", "اللوح 5", -51.9633, 197.793),
]

ALL_TASKS = LAMPS + CHARGERS + PANELS


# ======================= DroneAgent =======================

class DroneAgent:
    def __init__(self, name: str, system_addr: str, tasks: list, home_x: float, home_y: float):
        self.name = name
        self.addr = system_addr
        self.tasks = tasks
        self.home_x = home_x
        self.home_y = home_y

        self.drone: Optional[System] = None
        self.summary = DroneSummary(drone_name=name)

        self._last_n = None
        self._last_e = None
        self._ned_task = None

    def _log(self, msg: str):
        text = f"[{self.name}] {msg}"
        print(text)
        log_append(self.name, msg)

    # --- Telemetry ---
    async def _ned_listener(self):
        try:
            async for pv in self.drone.telemetry.position_velocity_ned():
                pos = pv.position
                self._last_n = pos.north_m
                self._last_e = pos.east_m
                await asyncio.sleep(0)
        except:
            return

    async def _wait_local_position(self):
        self._log("Waiting for local_position_ok…")
        async for health in self.drone.telemetry.health():
        # نستخدم فقط is_local_position_ok لأن is_attitude_ok غير موجود 
         if health.is_local_position_ok:
            break
        await asyncio.sleep(0.2)
        self._log("Local position OK.")


    # --- Offboard ---
    async def _start_offboard(self):
        init = PositionNedYaw(0, 0, -CRUISE_ALTITUDE, 0)
        await self.drone.offboard.set_position_ned(init)
        await self.drone.offboard.start()
        self._log("Offboard started, climbing…")
        await asyncio.sleep(PRE_TAKEOFF_HOLD_S)

    async def _fly_to(self, n, e, d, yaw, label):
        self._log(f"→ Fly to {label}: N={n:.1f}, E={e:.1f}, D={d:.1f}, yaw={yaw:.1f}")
        start = asyncio.get_event_loop().time()

        target = PositionNedYaw(n, e, d, yaw)
        while True:
            await self.drone.offboard.set_position_ned(target)

            if self._last_n is not None and self._last_e is not None:
                dist = math.hypot(self._last_n - n, self._last_e - e)
                print(f"[{self.name}] dist→{label}: {dist:.2f}m", end="\r")

                if dist <= ARRIVAL_RADIUS_M:
                    print()
                    self._log(f"Reached {label}")
                    return True

            if asyncio.get_event_loop().time() - start > ARRIVAL_TIMEOUT_S:
                print()
                self._log(f"⚠️ TIMEOUT {label}")
                return False

            await asyncio.sleep(0.2)

    # --- Camera ---
    async def _capture(self, label):
        try:
            await self.drone.camera.take_photo()
            self._log(f"📸 Photo at {label}")
            return True
        except:
            self._log(f"Camera error {label}")
            return False

    # --- Main Mission ---
    async def run(self):
        self.drone = System()
        self._log(f"Connecting to {self.addr} …")
        await self.drone.connect(system_address=self.addr)
        self._log("Connected.")

        self._ned_task = asyncio.create_task(self._ned_listener())
        await self._wait_local_position()

        self._log("Arming…")
        await self.drone.action.arm()

        await self._start_offboard()

        prev_n, prev_e = 0, 0

        for typ, label, wx, wy in self.tasks:
            rel_e = wx - self.home_x
            rel_n = wy - self.home_y

            self._log(f"Target {label}: rel N={rel_n:.1f}, E={rel_e:.1f}")

            yaw = yaw_from_vector(rel_n - prev_n, rel_e - prev_e)

            
            await self._fly_to(rel_n, rel_e, -CRUISE_ALTITUDE, yaw, f"{label}-cruise")

            
            await self._fly_to(rel_n, rel_e, -CAPTURE_ALTITUDE, yaw, f"{label}-cap")

            self._log(f"Holding {DWELL_SECONDS_PER_POINT}s…")
            await asyncio.sleep(DWELL_SECONDS_PER_POINT)

            flag = await self._capture(label)

            self.summary.add_record(
                VisitRecord(label, typ, rel_n, rel_e, flag)
            )

            await self._fly_to(rel_n, rel_e, -CRUISE_ALTITUDE, yaw, f"{label}-back")

            prev_n, prev_e = rel_n, rel_e

        # العودة للهوم
        yaw_home = yaw_from_vector(-prev_n, -prev_e)
        await self._fly_to(0, 0, -CRUISE_ALTITUDE, yaw_home, "HOME")

        await self.drone.offboard.stop()
        await self.drone.action.land()

        if self._ned_task:
            self._ned_task.cancel()

        self._log("\n" + self.summary.pretty())
        return self.summary


# ======================= main =======================

async def main():
    drone = DroneAgent(
        name="DRONE_1",
        system_addr="udp://:14540",
        tasks=ALL_TASKS,
        home_x=0.0,
        home_y=0.0
    )

    result = await drone.run()

    print("\n======================")
    print(result.pretty())
    print("======================")


if __name__ == "__main__":
    asyncio.run(main())
