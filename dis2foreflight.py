#!/usr/bin/env python3
"""
dis2foreflight.py - Listen for IEEE 1278.1 DIS (v6) Entity State PDUs and
forward an ownship (and optionally other entities as traffic) to ForeFlight.

ForeFlight input protocol used (the "X-Plane / simulator" UDP format, port 49002):
    XGPS<name>,<lon>,<lat>,<alt_m_msl>,<track_true_deg>,<groundspeed_m_s>
    XATT<name>,<heading_true_deg>,<pitch_deg>,<roll_deg>
    XTRAFFIC<name>,<id>,<lat>,<lon>,<alt_ft>,<vs_fpm>,<airborne 0/1>,<hdg_true>,<spd_kt>,<callsign>

The device can be on the same network (use its IP or the broadcast address),
or on another network reachable by unicast/routing (use its IP address).

Examples:
    # Follow entity 1:1:1, send to an iPad at 192.168.1.50
    python dis2foreflight.py --ownship 1:1:1 --ff-host 192.168.1.50

    # Auto-discover the iPad (ForeFlight broadcasts on UDP 63093 when on same LAN)
    python dis2foreflight.py --ownship 1:1:1 --discover

    # Join a multicast group, follow the first aircraft seen, also send traffic
    python dis2foreflight.py --mcast 239.1.2.3 --ff-host 192.168.1.50 --traffic

Only the standard library is required (Python 3.8+).
"""

import argparse
import json
import math
import select
import socket
import struct
import sys
import time

# ---------------------------------------------------------------- constants --
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

M_TO_FT = 3.280839895
MS_TO_KT = 1.943844492
MS_TO_FPM = 196.850394

PDU_TYPE_ENTITY_STATE = 1
ENTITY_STATE_MIN_LEN = 144          # fixed part, before articulation parameters
FF_DISCOVERY_PORT = 63093

# DIS entity kind / domain
KIND_PLATFORM = 1
DOMAIN_AIR = 2

HEADER = struct.Struct(">BBBBIHBB")   # ver, exercise, type, family, timestamp, length, status, pad


# ------------------------------------------------------------ DIS parsing ---
class EntityState:
    __slots__ = ("exercise", "site", "app", "entity", "force", "kind", "domain",
                 "vel", "ecef", "psi", "theta", "phi", "marking", "version")

    @property
    def key(self):
        return (self.site, self.app, self.entity)


def parse_entity_state(data, accept_versions):
    """Return an EntityState or None if the datagram is not a usable ESPDU."""
    if len(data) < ENTITY_STATE_MIN_LEN:
        return None
    ver, exercise, pdu_type, _family, _ts, length, _status, _pad = HEADER.unpack_from(data, 0)
    if pdu_type != PDU_TYPE_ENTITY_STATE:
        return None
    if accept_versions and ver not in accept_versions:
        return None
    if length and length > len(data):
        return None  # truncated

    es = EntityState()
    es.version = ver
    es.exercise = exercise
    es.site, es.app, es.entity = struct.unpack_from(">HHH", data, 12)
    es.force = data[18]
    # Entity type @20: kind(1) domain(1) country(2) category(1) subcat(1) specific(1) extra(1)
    es.kind, es.domain = data[20], data[21]
    # Linear velocity @36 (3 x float32, ECEF m/s)
    es.vel = struct.unpack_from(">3f", data, 36)
    # Location @48 (3 x float64, ECEF m)
    es.ecef = struct.unpack_from(">3d", data, 48)
    # Orientation @72 (psi, theta, phi float32 radians, relative to local NED)
    es.psi, es.theta, es.phi = struct.unpack_from(">3f", data, 72)
    # Marking @128: charset(1) + 11 chars
    raw = data[129:140]
    es.marking = raw.split(b"\x00", 1)[0].decode("ascii", "replace").strip()
    return es


# -------------------------------------------------------------- geodesy -----
def ecef_to_geodetic(x, y, z):
    """ECEF (m) -> (lat_rad, lon_rad, height_m above WGS84 ellipsoid)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    h = 0.0
    for _ in range(8):
        sl = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
        if abs(lat) < math.radians(80):
            h = p / math.cos(lat) - n
        else:
            h = z / sl - n * (1.0 - WGS84_E2)
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + h)))
    return lat, lon, h


def ecef_vel_to_enu(vx, vy, vz, lat, lon):
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    e = -so * vx + co * vy
    n = -sl * co * vx - sl * so * vy + cl * vz
    u = cl * co * vx + cl * so * vy + sl * vz
    return e, n, u


class Derived:
    """Geodetic position / kinematics derived from an EntityState."""

    def __init__(self, es):
        lat, lon, h = ecef_to_geodetic(*es.ecef)
        e, n, u = ecef_vel_to_enu(*es.vel, lat, lon)
        self.lat = math.degrees(lat)
        self.lon = math.degrees(lon)
        self.alt_m = h
        self.gs = math.hypot(e, n)                      # m/s
        self.track = math.degrees(math.atan2(e, n)) % 360.0
        self.vs = u                                     # m/s
        self.heading = math.degrees(es.psi) % 360.0
        self.pitch = math.degrees(es.theta)
        self.roll = math.degrees(es.phi)


# ------------------------------------------------------- ForeFlight output ---
def msg_xgps(name, d):
    return "XGPS%s,%.6f,%.6f,%.1f,%.1f,%.1f" % (name, d.lon, d.lat, d.alt_m, d.track, d.gs)


def msg_xatt(name, d):
    return "XATT%s,%.1f,%.1f,%.1f" % (name, d.heading, d.pitch, d.roll)


def msg_xtraffic(name, ident, es, d):
    callsign = (es.marking or "E%d" % es.entity)[:8]
    airborne = 1 if es.domain == DOMAIN_AIR else 0
    return "XTRAFFIC%s,%d,%.6f,%.6f,%.0f,%.0f,%d,%.1f,%.1f,%s" % (
        name, ident, d.lat, d.lon, d.alt_m * M_TO_FT, d.vs * MS_TO_FPM,
        airborne, d.heading, d.gs * MS_TO_KT, callsign)


# ------------------------------------------------------------------ main ----
def parse_entity_id(text):
    try:
        s, a, e = (int(p) for p in text.split(":"))
        return (s, a, e)
    except ValueError:
        raise argparse.ArgumentTypeError("entity id must look like SITE:APP:ENTITY, e.g. 1:1:1")


def main():
    
    ap = argparse.ArgumentParser(description="DIS Entity State -> ForeFlight UDP bridge")
    ap.add_argument("--listen-addr", default="0.0.0.0", help="local interface to bind (default 0.0.0.0)")
    ap.add_argument("--listen-port", type=int, default=3000, help="DIS UDP port (default 3000)")
    ap.add_argument("--mcast", help="multicast group to join for DIS (optional)")
    ap.add_argument("--mcast-iface", default="0.0.0.0", help="local interface IP for multicast join")
    ap.add_argument("--exercise", type=int, help="only accept this DIS exercise ID")
    ap.add_argument("--versions", default="6",
                    help="comma-separated accepted DIS protocol versions, or 'any' (default 6)")
    ap.add_argument("--ownship", type=parse_entity_id,
                    help="entity to report as ownship as SITE:APP:ENTITY (default: first air platform seen)")
    ap.add_argument("--ff-host", help="ForeFlight device IP or broadcast address")
    ap.add_argument("--ff-port", type=int, default=49002, help="ForeFlight sim port (default 49002)")
    ap.add_argument("--discover", default="discover",action="store_true",
                    help="learn the device IP from ForeFlight's UDP 63093 broadcast (same LAN only)")
    ap.add_argument("--name", default="DIS", help="simulator name shown in ForeFlight (default DIS)")
    ap.add_argument("--rate", type=float, default=5.0, help="output rate in Hz (default 5)")
    ap.add_argument("--traffic", action="store_true", help="send other air entities as XTRAFFIC")
    ap.add_argument("--max-traffic", type=int, default=30)
    ap.add_argument("--stale", type=float, default=5.0,
                    help="drop entities not updated for this many seconds (default 5)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()


    if not args.ff_host and not args.discover:
        ap.error("give --ff-host and/or --discover")


    accept_versions = None if args.versions.lower() == "any" else {
        int(v) for v in args.versions.split(",")}

    # DIS receive socket
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    rx.bind((args.listen_addr, args.listen_port))
    if args.mcast:
        mreq = socket.inet_aton(args.mcast) + socket.inet_aton(args.mcast_iface)
        rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    socks = [rx]

    # ForeFlight discovery socket
    disc = None
    if args.discover:
        disc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        disc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                disc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass
        disc.bind(("0.0.0.0", FF_DISCOVERY_PORT))
        socks.append(disc)

    # Output socket
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    ff_host = args.ff_host
    ownship_key = args.ownship
    entities = {}        # key -> (EntityState, Derived, last_rx_time)
    traffic_ids = {}     # key -> small int id
    period = 1.0 / max(args.rate, 0.1)
    next_send = time.monotonic()
    pdu_count = 0

    print("Listening for DIS on %s:%d%s -> ForeFlight %s:%d"
          % (args.listen_addr, args.listen_port,
             " (mcast %s)" % args.mcast if args.mcast else "",
             ff_host or "(discovering)", args.ff_port), file=sys.stderr)

    try:
        while True:
            timeout = max(0.0, next_send - time.monotonic())
            readable, _, _ = select.select(socks, [], [], timeout)
            now = time.monotonic()

            for s in readable:
                data, addr = s.recvfrom(65535)
                if s is disc:
                    try:
                        info = json.loads(data.decode("utf-8", "ignore"))
                    except ValueError:
                        continue
                    if info.get("App") == "ForeFlight" and ff_host != addr[0]:
                        ff_host = addr[0]
                        print("Discovered ForeFlight at %s" % ff_host, file=sys.stderr)
                    continue

                es = parse_entity_state(data, accept_versions)
                if es is None:
                    continue
 #               else:
 #                   print(es)
                if args.exercise is not None and es.exercise != args.exercise:
                    continue
                pdu_count += 1
                entities[es.key] = (es, Derived(es), now)
                if ownship_key is None and es.kind == KIND_PLATFORM and es.domain == DOMAIN_AIR:
                    ownship_key = es.key
                    print("Ownship = entity %d:%d:%d %s" % (*es.key, es.marking), file=sys.stderr)

            if now < next_send:
                continue
            next_send = now + period

            # expire stale entities
            for k in [k for k, v in entities.items() if now - v[2] > args.stale]:
                del entities[k]

            if not ff_host:
                continue
            dest = (ff_host, args.ff_port)

            own = entities.get(ownship_key) if ownship_key else None
            if own:
                _, d, _ = own
                for m in (msg_xgps(args.name, d), msg_xatt(args.name, d)):
                    tx.sendto(m.encode("ascii"), dest)
                    if args.verbose:
                        print(m)

            if args.traffic:
                sent = 0
                for k, (es, d, _) in entities.items():
                    if k == ownship_key or es.domain != DOMAIN_AIR:
                        continue
                    ident = traffic_ids.setdefault(k, len(traffic_ids) + 1)
                    m = msg_xtraffic(args.name, ident, es, d)
                    tx.sendto(m.encode("ascii"), dest)
                    if args.verbose:
                        print(m)
                    sent += 1
                    if sent >= args.max_traffic:
                        break
    except KeyboardInterrupt:
        print("\nStopped. %d Entity State PDUs processed." % pdu_count, file=sys.stderr)


if __name__ == "__main__":
    main()