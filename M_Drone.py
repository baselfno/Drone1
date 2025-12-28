# -*- coding: utf-8 -*-
# Drone.py — Basel SmartU (3 Drones via MAVSDK, async, world→GPS mapping)
#
# تشغيل PX4 كما هو عندك (لا تغيّره):
# 1) PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=SmartU PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4
# 2) PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=SmartU PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE="10,0,1" PX4_SIM_MAVSDK_UDP_PORT=14541 PX4_SIM_QGC_UDP_PORT=14551 ./build/px4_sitl_default/bin/px4 -i 1
# 3) PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=SmartU PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE="-10,0,1" PX4_SIM_MAVSDK_UDP_PORT=14542 PX4_SIM_QGC_UDP_PORT=14552 ./build/px4_sitl_default/bin/px4 -i 2

import asyncio
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional, Tuple

from mavsdk import System
from mavsdk.camera import CameraError

# --------------------------- إعدادات عامة ---------------------------
CRUISE_ALTITUDE = 35.0      # ارتفاع الطيران بين الأهداف (م فوق الـMSL)
CAPTURE_ALTITUDE = 15.0     # ارتفاع التصوير عند الهدف (م فوق الـMSL)
PRE_TAKEOFF_HOLD_S = 10     # الانتظار بعد الإقلاع قبل أول مهمة
DWELL_SECONDS_PER_POINT = 15
ARRIVAL_RADIUS_M = 5.0      # توسيع نصف قطر الوصول (متر)
ARRIVAL_TIMEOUT_S = 240.0
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# --------------------------- أدوات مساعدة ---------------------------
def now_ts() -> str:
    return datetime.utcnow().strftime("%Y-%m-%d_%H-%M-%S")

def log_append(stem: str, line: str):
    fpath = LOG_DIR / f"{stem}.log"
    with open(fpath, "a", encoding="utf-8") as f:
        f.write(f"{now_ts()} | " + line + "\n")

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

def enu_to_geo(home_lat: float, home_lon: float, east_m: float, north_m: float) -> Tuple[float, float]:
    """تحويل إزاحة (شرق/شمال بالمتر) حول الهوم إلى lat/long تقريبية."""
    R = 6378137.0
    dlat = (north_m / R) * (180.0 / math.pi)
    dlon = (east_m / (R * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)
    return home_lat + dlat, home_lon + dlon

# --------------------------- نماذج البيانات ---------------------------
@dataclass
class VisitRecord:
    label: str
    type: str
    lat: float
    lon: float
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
                f"{rec.label} ({rec.type}) @ ({rec.lat:.7f},{rec.lon:.7f}) {rec.note}"
            )
        return "\n".join(lines)

# --------------------------- بيانات الأهداف (World X,Y) ---------------------------
# ملاحظة: X = East (m), Y = North (m) بالنسبة لنقطة أصل العالم (0,0) في SmartU

LAMPS = [
    ("لمبة 1", 207.000000, 155.000000, -1.0),
    ("لمبة 2",  19.715800, 105.860000,  0.0),
    ("لمبة 3",   5.317170, 104.462000,  0.0),
    ("لمبة 4",  18.763700, 122.364000,  0.0),
    ("لمبة 5", 524.973000,-141.123000,  0.0),
    ("لمبة 6", 411.443000,-132.508000,  0.0),
    ("لمبة 7", 349.778000,-129.297000,  0.0),
    ("لمبة 8", 288.595000,-138.744000,  0.0),
]

CHARGERS = [
    ("محطة شحن 1", 207.000000, 155.000000, -1.0),
    ("محطة شحن 2",-132.232000,  99.531900,  0.0),
    ("محطة شحن 3",-108.298000, 111.632000,  0.0),
    ("محطة شحن 4", -96.828700,  89.922800,  0.0),
]

PANELS = [
    ("اللوح 1", -50.850000,203.759000,3.809150),
    ("اللوح 2", -47.206300,201.240000,3.970000),
    ("اللوح 3", -43.233400,205.607000,4.060480),
    ("اللوح 4", -39.328100,203.148000,3.970680),
    ("اللوح 5", -51.963300,197.793000,3.970680),
]

# توزيع حسب النوع:
DRONE1_TASKS = [("charger", *c) for c in CHARGERS]  # محطات الشحن
DRONE2_TASKS = [("lamp", *l)    for l in LAMPS]     # اللمبات
DRONE3_TASKS = [("panel", *p)   for p in PANELS]    # الألواح

# --------------------------- DroneAgent ---------------------------
class DroneAgent:
    def __init__(self,
                 name: str,
                 system_addr: str,
                 world_tasks: List[Tuple[str,str,float,float,float]]):
        """
        world_tasks: (type, label, world_x, world_y, world_z)
        type ∈ {"lamp","charger","panel"} للتقرير فقط.
        """
        self.name = name
        self.addr = system_addr
        self.world_tasks = world_tasks
        self.drone: Optional[System] = None
        self.summary = DroneSummary(drone_name=name)

    # ---------- Logging ----------
    def _log(self, msg: str):
        text = f"[{self.name}] {msg}"
        print(text)
        log_append(self.name, msg)

    # ---------- Telemetry ----------
    async def _wait_until_reached(self, tgt_lat, tgt_lon,
                                  radius_m=ARRIVAL_RADIUS_M,
                                  timeout_s=ARRIVAL_TIMEOUT_S):
        start = asyncio.get_event_loop().time()
        while True:
            p = await anext(self.drone.telemetry.position())
            dist = haversine_m(p.latitude_deg, p.longitude_deg, tgt_lat, tgt_lon)
            print(f"[{self.name}] dist→target: {dist:7.2f} m", end="\r")
            if dist <= radius_m:
                print()
                return
            if asyncio.get_event_loop().time() - start > timeout_s:
                print()
                self._log("TIMEOUT REACHING TARGET")
                return
            await asyncio.sleep(0.3)

    async def _wait_for_gps_and_home(self):
        """انتظار جاهزية GPS و Home قبل بدء المهمة."""
        self._log("Waiting for GPS & Home position…")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
            await asyncio.sleep(0.5)

        home = await anext(self.drone.telemetry.home())
        self._log("GPS & Home position OK.")
        return home

    # ---------- Camera ----------
    async def _capture_with_flag(self, label: str) -> bool:
        try:
            await self.drone.camera.take_photo()
            self._log(f"📸 Captured at {label}")
            return True
        except CameraError as e:
            self._log(f"Camera error at {label}: {e}")
            return False
        except Exception as e:
            self._log(f"Camera unknown error at {label}: {e}")
            return False

    # ---------- Mission ----------
    async def run(self):
        self.drone = System()
        self._log(f"Connecting to {self.addr} …")
        await self.drone.connect(system_address=self.addr)
        self._log("Connected.")

        # GPS + Home
        home = await self._wait_for_gps_and_home()
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        home_alt = home.absolute_altitude_m

        cruise_alt = home_alt + CRUISE_ALTITUDE
        capture_alt = home_alt + CAPTURE_ALTITUDE

        self._log(f"Home lat={home_lat:.7f}, lon={home_lon:.7f}, altMSL={home_alt:.1f}")
        self._log(f"Cruise Alt MSL = {cruise_alt:.1f}, Capture Alt MSL = {capture_alt:.1f}")

        # إقلاع
        self._log("Arming…")
        await self.drone.action.arm()
        self._log("Taking off…")
        await self.drone.action.takeoff()
        await asyncio.sleep(PRE_TAKEOFF_HOLD_S)

        prev_lat, prev_lon = home_lat, home_lon

        # تنفيذ الأهداف
        for idx, (typ, label, world_x, world_y, _world_z) in enumerate(self.world_tasks, start=1):
            # نعامل X,Y كإحداثيات العالم بالنسبة للأصل (0,0)
            east  = world_x
            north = world_y

            tgt_lat, tgt_lon = enu_to_geo(home_lat, home_lon, east, north)
            self._log(
                f"WP{idx} {label}: world({world_x:.1f},{world_y:.1f}) "
                f"→ ENU({east:.1f},{north:.1f}) "
                f"→ gps({tgt_lat:.7f},{tgt_lon:.7f})"
            )

            yaw = bearing_deg(prev_lat, prev_lon, tgt_lat, tgt_lon)

            # مرحلة 1: الطيران على 35 م
            await self.drone.action.goto_location(tgt_lat, tgt_lon, cruise_alt, yaw)
            await self._wait_until_reached(tgt_lat, tgt_lon)

            # مرحلة 2: النزول إلى 15 م للتصوير
            await self.drone.action.goto_location(tgt_lat, tgt_lon, capture_alt, yaw)
            await asyncio.sleep(3.0)

            self._log(f"[ARRIVED] Holding {DWELL_SECONDS_PER_POINT}s at {label}")
            await asyncio.sleep(DWELL_SECONDS_PER_POINT)

            flag = await self._capture_with_flag(label)
            note = "(ok)" if flag else "(capture failed)"

            self.summary.add_record(
                VisitRecord(label=label, type=typ,
                            lat=tgt_lat, lon=tgt_lon,
                            photo_flag=flag, note=note)
            )

            # مرحلة 3: الرجوع لارتفاع 35 م
            await self.drone.action.goto_location(tgt_lat, tgt_lon, cruise_alt, yaw)
            await asyncio.sleep(3.0)

            prev_lat, prev_lon = tgt_lat, tgt_lon

        # العودة للبيت والهبوط
        yaw_home = bearing_deg(prev_lat, prev_lon, home_lat, home_lon)
        self._log("Returning Home…")
        await self.drone.action.goto_location(home_lat, home_lon, cruise_alt, yaw_home)
        await self._wait_until_reached(home_lat, home_lon)

        self._log("Landing…")
        await self.drone.action.land()

        self._log("\n" + self.summary.pretty())
        return self.summary

# --------------------------- main ---------------------------
async def main():
    d1 = DroneAgent("DRONE_1", "udp://:14540", DRONE1_TASKS)  # محطات الشحن
    d2 = DroneAgent("DRONE_2", "udp://:14541", DRONE2_TASKS)  # اللمبات
    d3 = DroneAgent("DRONE_3", "udp://:14542", DRONE3_TASKS)  # الألواح

    results = await asyncio.gather(
        d1.run(),
        d2.run(),
        d3.run(),
        return_exceptions=True
    )

    print("\n" + "="*70)
    print("🔚 ALL DRONES — CONSOLIDATED REPORT")
    for res in results:
        if isinstance(res, DroneSummary):
            print(res.pretty())
            print("-"*70)
        else:
            print(f"⚠️ Drone returned error: {res}")
            print("-"*70)

if __name__ == "__main__":
    asyncio.run(main())
