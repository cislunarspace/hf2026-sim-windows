"""Engine lock-selection debug probe (stand-only, COOP_EVAL_DEBUG=1).

For each UAV per tick, record the published gimbal FOV, the gimbal aim,
the engine-matched entity, and the angular offset of every real target and
the nearest decoy from the gimbal boresight. Answers empirically:

  * whether runtime ``set_fov`` changes the published (and detection) FOV;
  * which target the engine locks when several sit inside the cone;
  * how often a real target is inside the cone while the engine locks a
    decoy instead (lock-selection model).

Writes ``output/eval_debug2.csv`` (one row per UAV per tick).
"""
from __future__ import annotations

import math

from .._vendored.geometry import bearing_deg, haversine_m  # type: ignore


def _tilt_of(uav, lat: float, lon: float, alt: float = 0.0):
    """Gimbal tilt that would aim at (lat, lon): body pan + tilt + range.

    Mirrors ``algorithms.tracking.gimbal.compute_gimbal_angles``:
    pan is body-relative (0 = nose, right positive), tilt negative = down.
    """
    pan = (bearing_deg(uav.lat, uav.lon, lat, lon) - uav.heading + 180.0) % 360.0 - 180.0
    gnd = haversine_m(uav.lat, uav.lon, lat, lon)
    alt_diff = uav.alt - alt
    if gnd < 1e-3:
        tilt = -90.0 if alt_diff > 0 else 0.0
    else:
        tilt = -math.degrees(math.atan2(alt_diff, gnd))
    return pan, tilt, gnd


def _sep_deg(pan_g: float, tilt_g: float, pan_t: float, tilt_t: float) -> float:
    """Angular separation between gimbal boresight and a target LOS (deg)."""
    dpan = math.radians(pan_t - pan_g)
    c = (math.sin(math.radians(tilt_g)) * math.sin(math.radians(tilt_t))
         + math.cos(math.radians(tilt_g)) * math.cos(math.radians(tilt_t))
         * math.cos(dpan))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def dump_lock_debug(ws, uav_map, uavs, out) -> None:
    """Append one U2 row per UAV to ``out`` (already-open text file)."""
    try:
        from competition.user_algorithms.coop_decoy.agent import AGENT_DEBUG_STATE
    except Exception:  # noqa: BLE001
        AGENT_DEBUG_STATE = {}
    raw = {u.uid: u for u in uavs}
    true_pos = {uid: (e.lat, e.lon, 0.0) for uid, e in ws.targets.items()}
    decoy_pos = {uid: (e.lat, e.lon, 0.0) for uid, e in ws.decoys.items()}
    for uid, e in sorted(ws.uavs.items()):
        gim = e.raw.get("gimbal_tracking", {}) or {}
        det = gim.get("detection", {}) or {}
        tpos = det.get("target_position")
        pan_g = float(gim.get("pan_angle", 0.0))
        tilt_g = float(gim.get("tilt_angle", 0.0))
        fov_pub = float(gim.get("fov", gim.get("fov_deg", 30.0)))
        detected = bool(det.get("detected", False))
        conf = float(det.get("confidence", 0.0))
        misid = bool(det.get("misid_flag", False))
        m = uav_map.get(uid)
        matched_uid = m.target_uid if m and m.is_effective else (m.decoy_uid if m else None)
        matched_eff = int(bool(m and m.is_effective))
        matched_is_decoy = int(bool(m and m.was_misid))
        # matched entity view (real target or decoy position)
        matched_sep = matched_range = ""
        if matched_uid:
            if matched_is_decoy and matched_uid in decoy_pos:
                mp = decoy_pos[matched_uid]
            elif not matched_is_decoy and matched_uid in true_pos:
                mp = true_pos[matched_uid]
            else:
                mp = None
            if mp:
                p, t, r = _tilt_of(e, mp[0], mp[1], mp[2])
                matched_sep = f"{_sep_deg(pan_g, tilt_g, p, t):.2f}"
                matched_range = f"{r:.0f}"
        # every real target + nearest decoy vs boresight
        n_true = len(true_pos)
        n_true_in = 0
        nearest_true_sep = nearest_true_range = ""
        best_true = None
        half_live = fov_pub / 2.0
        for tp in true_pos.values():
            p, t, r = _tilt_of(e, tp[0], tp[1], tp[2])
            sep = _sep_deg(pan_g, tilt_g, p, t)
            if sep <= half_live:
                n_true_in += 1
            if best_true is None or sep < best_true[0]:
                best_true = (sep, r)
        if best_true:
            nearest_true_sep = f"{best_true[0]:.2f}"
            nearest_true_range = f"{best_true[1]:.0f}"
        n_decoy_in = 0
        nearest_decoy_sep = nearest_decoy_range = ""
        best_decoy = None
        for dp in decoy_pos.values():
            p, t, r = _tilt_of(e, dp[0], dp[1], dp[2])
            sep = _sep_deg(pan_g, tilt_g, p, t)
            if sep <= half_live:
                n_decoy_in += 1
            if best_decoy is None or sep < best_decoy[0]:
                best_decoy = (sep, r)
        if best_decoy:
            nearest_decoy_sep = f"{best_decoy[0]:.2f}"
            nearest_decoy_range = f"{best_decoy[1]:.0f}"
        st = AGENT_DEBUG_STATE.get(uid, {})
        out.write(
            f"U2,{ws.sim_time:.3f},{uid},{fov_pub:.1f},{pan_g:.1f},{tilt_g:.1f},"
            f"{int(detected)},{conf:.3f},{int(misid)},{matched_uid},"
            f"{matched_eff},{matched_is_decoy},{matched_sep},{matched_range},"
            f"{n_true},{n_true_in},{n_decoy_in},"
            f"{nearest_true_sep},{nearest_true_range},"
            f"{nearest_decoy_sep},{nearest_decoy_range},"
            f"{e.lat:.6f},{e.lon:.6f},{e.alt:.1f},{e.heading:.1f},"
            f"{st.get('state','')},{st.get('wingman','')},{st.get('confirmed','')},"
            f"{st.get('tgt','')},{st.get('shared','')},{st.get('leader_age','')},"
            f"{st.get('dwell','')}\n"
        )
