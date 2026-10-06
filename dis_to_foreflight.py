import socket
import struct
import math
from pyproj import Transformer

# --- KONFIGURATION ---
DIS_PORT = 3000          # Standardport där ditt simuleringsnätverk skickar DIS-trafik
FOREFLIGHT_PORT = 49002  # Standardport som ForeFlight lyssnar på för simulator-data
FOREFLIGHT_IP = "255.255.255.255"  # Skickar som broadcast (eller sätt iPadens specifika IP)

# Initiera koordinattransformator (Från ECEF till WGS84 Geodetisk)
# ECEF = EPSG:4978, WGS84 (Lat, Lon, Höjd) = EPSG:4979
transformer = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)

def ecef_to_lla(x, y, z):
    """Konverterar ECEF (meter) till Longitud, Latitud, Höjd (meter)"""
    lon, lat, alt = transformer.transform(x, y, z)
    return lat, lon, alt

def parse_dis_v6_entity_state(data):
    """
    Parsar en binär DIS v6 Entity State PDU enligt IEEE 1278.1-1995.
    Returnerar en ordbok med relevant data eller None om paketet är ogiltigt.
    """
    if len(data) ,<lon>,<lat>,<höjd_msl_m>,<kurs>,<hastighet_m_s>
                xgps_str = f"XGPS{entity_state['name']},{entity_state['lon']:.6f},{entity_state['lat']:.6f},{entity_state['alt_m']:.2f},{entity_state['heading']:.1f},{entity_state['speed_ms']:.1f}"
                
                # 2. Bygg ForeFlight Attityd-sträng (XATT)
                # Format: XATT<Namn>,<kurs>,<pitch>,<roll>
                xatt_str = f"XATT{entity_state['name']},{entity_state['heading']:.1f},{entity_state['pitch']:.1f},{entity_state['roll']:.1f}"

                # Skicka paketen till ForeFlight via UDP
                ff_socket.sendto(xgps_str.encode('utf-8'), (FOREFLIGHT_IP, FOREFLIGHT_PORT))
                ff_socket.sendto(xatt_str.encode('utf-8'), (FOREFLIGHT_IP, FOREFLIGHT_PORT))

                print(f"Skickat -> {entity_state['name']}: Lat: {entity_state['lat']:.4f}, Lon: {entity_state['lon']:.4f}, Alt: {entity_state['alt_m']:.0f}m, Hdg: {entity_state['heading']:.1f}°")

    except KeyboardInterrupt:
        print("\nStänger ner bryggan...")
    finally:
        dis_socket.close()
        ff_socket.close()

if __name__ == '__main__':
    main()
