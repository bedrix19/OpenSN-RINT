
import math
import ephem
import datetime
from opensn.const.const_var import R_EARTH,LIGHT_SPEED_M_S
from opensn.model.position import Position
from instance_types import TYPE_SATELLITE,TYPE_GROUND_STATION
from instance_types import EX_TLE0_KEY,EX_TLE1_KEY,EX_TLE2_KEY,EX_LATITUDE_KEY,EX_LONGITUDE_KEY,EX_ALTITUDE_KEY
from opensn.model.instance import Instance

def deg2rad(deg: float) -> float:
    return deg / 180 * math.pi
        
def calculate_postion(instance: Instance,time:datetime.datetime) -> Position:
    ret = Position()
    if instance.type == TYPE_SATELLITE and instance.start:
        ephem_time = ephem.Date(time)
        ephem_obj = ephem.readtle(
            instance.extra[EX_TLE0_KEY],
            instance.extra[EX_TLE1_KEY],
            instance.extra[EX_TLE2_KEY],
        )
        ephem_obj.compute(ephem_time)
        ret.latitude = ephem_obj.sublat
        ret.longitude = ephem_obj.sublong
        ret.altitude = ephem_obj.elevation
    elif instance.type == TYPE_GROUND_STATION:
        ret.latitude = deg2rad(float(instance.extra[EX_LATITUDE_KEY]))
        ret.longitude = deg2rad(float(instance.extra[EX_LONGITUDE_KEY]))
        ret.altitude = deg2rad(float(instance.extra[EX_ALTITUDE_KEY]))
    return ret

def distance_meter(one:Position,another:Position) -> float: # meter
    z1 = (one.altitude+R_EARTH) * math.sin(one.latitude)
    base1 = (one.altitude+R_EARTH) * math.cos(one.latitude)
    x1 = base1 * math.cos(one.longitude)
    y1 = base1 * math.sin(one.longitude)
    z2 = (another.altitude+R_EARTH) * math.sin(another.latitude)
    base2 = (another.altitude+R_EARTH) * math.cos(another.latitude)
    x2 = base2 * math.cos(another.longitude)
    y2 = base2 * math.sin(another.longitude)
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

def get_propagation_delay_s(distance_meter:float) -> float: # second
    return distance_meter / LIGHT_SPEED_M_S

def select_closest_satellite(
        ground_station:Instance,
        position_map:dict[str,Position],
        instance_map:dict[str,Instance]
    ) -> (str,bool) :
    closet_distance = math.inf
    select_satellite_id = ""
    change = True
    for instance_id,instance_info in instance_map.items():
        if instance_info.type != TYPE_SATELLITE:
            continue
        new_distance = distance_meter(
            position_map[instance_id],
            position_map[ground_station.instance_id],
        )
        if new_distance < closet_distance:
            closet_distance = new_distance
            select_satellite_id = instance_id
    if len(ground_station.connections) < 0 and select_satellite_id == "":
        return "",False
    
    for end_info in ground_station.connections.values():
        if select_satellite_id == end_info.instance_id:
            change = False
    return select_satellite_id,change

# φ is latitude, λ is longitude, θ is the bearing (clockwise from north),
# δ is the angular distance d/R; d being the distance travelled,
# R the earth’s radius

EARTH_RADIUS_M = 6_371_000.0

def move_ground_position(
    lat_deg:     float,
    lon_deg:     float,
    speed_mps:   float,
    heading_deg: float,
    dt_seconds:  float,
) -> tuple[float, float]:
    """
    Advance a point on the Earth's surface at constant speed and bearing.
    Returns (new_lat_deg, new_lon_deg).
    """
    φ1    = math.radians(lat_deg)
    λ1    = math.radians(lon_deg)
    θ     = math.radians(heading_deg)
    δ     = (speed_mps * dt_seconds) / EARTH_RADIUS_M  # angular distance [rad]

    φ2 = math.asin(
        math.sin(φ1) * math.cos(δ) +
        math.cos(φ1) * math.sin(δ) * math.cos(θ)
    )
    λ2 = λ1 + math.atan2(
        math.sin(θ) * math.sin(δ) * math.cos(φ1),
        math.cos(δ) - math.sin(φ1) * math.sin(φ2),
    )
    # Normalise longitude to [-180, +180] in case we cross the antimeridian
    λ2 = (math.degrees(λ2) + 540) % 360 - 180

    return math.degrees(φ2), λ2

def move_TU_position(
    lat_deg:     float,
    lon_deg:     float,
    speed_mps:   float,
    heading_deg: float,
    dt_seconds:  float,
    user_altitude: float,
) -> tuple[float, float]:
    """
    Advance a point on the Earth's surface at constant speed and bearing.
    Returns (new_lat_deg, new_lon_deg).
    """
    φ1    = math.radians(lat_deg)
    λ1    = math.radians(lon_deg)
    θ     = math.radians(heading_deg)
    δ     = (speed_mps * dt_seconds) / (EARTH_RADIUS_M + user_altitude)  # angular distance

    φ2 = math.asin(
        math.sin(φ1) * math.cos(δ) +
        math.cos(φ1) * math.sin(δ) * math.cos(θ)
    )
    λ2 = λ1 + math.atan2(
        math.sin(θ) * math.sin(δ) * math.cos(φ1),
        math.cos(δ) - math.sin(φ1) * math.sin(φ2),
    )
    # Normalise longitude to [-180, +180] in case we cross the antimeridian
    λ2 = (math.degrees(λ2) + 540) % 360 - 180

    return math.degrees(φ2), λ2

def bearing_to(lat1_deg: float, lon1_deg: float,
               lat2_deg: float, lon2_deg: float) -> float:
    
    φ1, φ2 = math.radians(lat1_deg), math.radians(lat2_deg)
    Δλ = math.radians(lon2_deg - lon1_deg)
    x = math.sin(Δλ) * math.cos(φ2)
    y = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(Δλ)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def haversine_m(lat1_deg: float, lon1_deg: float,
                lat2_deg: float, lon2_deg: float) -> float:
    
    φ1, φ2 = math.radians(lat1_deg), math.radians(lat2_deg)
    Δφ = math.radians(lat2_deg - lat1_deg)
    Δλ = math.radians(lon2_deg - lon1_deg)
    a = math.sin(Δφ/2)**2 + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ/2)**2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))