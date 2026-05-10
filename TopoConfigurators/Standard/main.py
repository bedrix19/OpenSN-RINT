from opensn.operator.emulator_operator import EmulatorOperator
from opensn.model.instance import Instance
from opensn.model.position import Position
from opensn.const.dict_fields import PARAMETER_KEY_CONNECT,PARAMETER_KEY_DELAY,PARAMETER_KEY_BANDWIDTH,PARAMETER_KEY_LOSS
from opensn.model.link import LinkBase
from opensn.utils.tools import dec2ra
from config import ADDR,PORT
from datetime import datetime
from trajectory import move_ground_position, move_TU_position, bearing_to, haversine_m, calculate_postion,distance_meter,select_closest_satellite,get_propagation_delay_s
from instance_types import TYPE_GROUND_STATION, TYPE_SATELLITE, EX_ORBIT_INDEX,EX_ALTITUDE_KEY,EX_LATITUDE_KEY,EX_LONGITUDE_KEY, EX_AREA_KEY
from address_type import LINK_V4_ADDR_KEY
from time import sleep
from address_allocator import alloc_ipv4,format_ipv4
from loguru import logger
import json, math
step_second = 2

# config para el TU
TU_WAYPOINTS = [
    # overwrite the start position from the json
    (40.4722, -3.6870), # Madrid Ch
    (41.5034, -5.7425), # Zamora
    (42.3687, -7.8641), # Ourense
    (42.8781, -8.5448), # Santiago de Compostela
    (42.5947, -8.7654), # Vilagarcia de Arousa
    (42.4260, -8.6478), # Pontevedra
    (42.2328, -8.7226), # Vigo Urzaiz
]

TU_ROLE             = "tu_gs"   # must match with extra["role"] from JSON
TU_SPEED_MPS        = 750       # plane: ~250 m/s, car: ~30 m/s, ship: ~10 m/s, ave-renfe: ~83.3 m/s
TU_WAYPOINT_RADIUS  = 5_000.0   # metros - distancia para considerar waypoint alcanzado
TU_ALTITUDE         = 50_000.0 #10_000.0
tu_waypoint_index   = 0
TU_HEADING_DEG      = 45.0      # 0=North, 90=East, 180=South, 270=West

tu_current_lat: float | None = None
tu_current_lon: float | None = None

polar_threshold = dec2ra(66.5)

def genenrate_config(cli:EmulatorOperator,node_index:int,instance_id:str):
    instance_info = cli.get_instance(node_index,instance_id)
    config_map = {
        "instance_id": instance_id,
        "link_infos": {},
        "end_infos": {},
    }
    if instance_info.type == TYPE_SATELLITE:
        config_map['area'] = instance_info.extra[EX_AREA_KEY]
    for k,v in instance_info.connections.items():
        instance_index = -1
        link_info = cli.get_link(node_index,k)
        for end_index in range(len(link_info.end_infos)):
            if link_info.end_infos[end_index].instance_id == instance_id:
                instance_index = end_index
        if instance_index < 0:
            return {}
        another_instance_info = cli.get_instance(link_info.end_infos[1-instance_index].end_node_index,link_info.end_infos[1-instance_index].instance_id)
        config_map["link_infos"][k] = link_info.address_infos[instance_index]
        config_map["end_infos"][k] = {
            "instance_id": v.instance_id,
            "type": v.instance_type,
        }
        if another_instance_info.type == TYPE_SATELLITE:
            config_map["end_infos"][k]['area'] = another_instance_info.extra[EX_AREA_KEY]
    return config_map

if __name__ == "__main__":


    instance_config_updated:dict[str,str] = {}
    
    cli = EmulatorOperator(ADDR,PORT)


    gs_updated_ids: set[str] = set()    # iterated nodes
    # Create Emulator Operator
    while True:
        node_list = cli.get_node_map()
        all_instance_map: dict[str,Instance] = {}
        node_link_map: dict[int,dict[str,LinkBase]] = {}
        ground_station_list:list[Instance] = []
        gs_updated_ids.clear()


        for node_index,node in node_list.items():
            instance_map = cli.get_instance_map(node_index)
            for instance_id,instance in instance_map.items():
                all_instance_map[instance_id] = instance
                if instance.type == TYPE_GROUND_STATION:
                    ground_station_list.append(instance)
                    gs_position = Position()


                    role = instance.extra.get("role", "")
                    if role == TU_ROLE:
                        if tu_current_lat is None:
                            tu_current_lat, tu_current_lon = TU_WAYPOINTS[0]
                            logger.info(f"[TU] Initialized at lat={tu_current_lat:.4f} lon={tu_current_lon:.4f}")

                        # Comprobar si hemos llegado al waypoint actual
                        if tu_waypoint_index < len(TU_WAYPOINTS) - 1:
                            target_lat, target_lon = TU_WAYPOINTS[tu_waypoint_index + 1]
                            dist_to_next = haversine_m(tu_current_lat, tu_current_lon,
                                                    target_lat, target_lon)
                            if dist_to_next < TU_WAYPOINT_RADIUS:
                                tu_waypoint_index += 1
                                logger.info(f"[TU] Reached waypoint {tu_waypoint_index}: {TU_WAYPOINTS[tu_waypoint_index]}")

                        # Moverse hacia el siguiente waypoint (o quedarse si ya llegamos)
                        if tu_waypoint_index < len(TU_WAYPOINTS) - 1:
                            target_lat, target_lon = TU_WAYPOINTS[tu_waypoint_index + 1]
                            heading = bearing_to(tu_current_lat, tu_current_lon,
                                                target_lat, target_lon)
                            tu_current_lat, tu_current_lon = move_TU_position(
                                lat_deg     = tu_current_lat,
                                lon_deg     = tu_current_lon,
                                speed_mps   = TU_SPEED_MPS,
                                heading_deg = heading,          # recalculado cada paso
                                dt_seconds  = step_second,
                                user_altitude = TU_ALTITUDE,
                            )
                            logger.info(f"[TU] → waypoint {tu_waypoint_index+1} "
                                        f"({dist_to_next/1000:.1f} km) | "
                                        f"bearing={heading:.1f}° | "
                                        f"pos=({tu_current_lat:.4f}, {tu_current_lon:.4f})")
                        else:
                            logger.info("[TU] Destination reached: Vigo Urzáiz")
                        instance.extra[EX_LATITUDE_KEY] = str(tu_current_lat)
                        instance.extra[EX_LONGITUDE_KEY] = str(tu_current_lon)
                        instance.extra[EX_ALTITUDE_KEY]  = str(TU_ALTITUDE)

                        # Try to persist to etcd (best effort)
                        try:
                            cli.put_instance(instance)
                            logger.debug(f"[TU] put_instance OK for {instance_id}")
                        except Exception as e:
                            logger.warning(f"[TU] put_instance failed: {e} — position still updated via put_position")

                        gs_position.latitude  = math.radians(tu_current_lat)
                        gs_position.longitude = math.radians(tu_current_lon)
                        gs_position.altitude  = math.radians(float(instance.extra[EX_ALTITUDE_KEY]))
                    else:
                        gs_position.latitude  = float(instance.extra[EX_LATITUDE_KEY]) / 180 * math.pi
                        gs_position.longitude = float(instance.extra[EX_LONGITUDE_KEY]) / 180 * math.pi
                        gs_position.altitude  = math.radians(float(instance.extra[EX_ALTITUDE_KEY]))
                    
                    # This is what the visualizer reads — must be called with correct values
                    cli.put_position(instance_id, gs_position)
                    logger.debug(f"[GS] put_position called: id={instance_id} lat={gs_position.latitude:.5f} lon={gs_position.longitude:.5f}")
                    gs_updated_ids.add(instance_id)


        address_map = {}
        for node_index,node in node_list.items():
            node_link_map[node_index] = {}
            link_map = cli.get_link_map(node_index)
            for link_id,link_info in link_map.items():
                if LINK_V4_ADDR_KEY not in link_info.address_infos[0] or \
                    LINK_V4_ADDR_KEY not in link_info.address_infos[1] is None:
                    if link_id not in address_map.keys():
                        address_map[link_id] = alloc_ipv4(30)
                    
                    subnet = address_map[link_id]
                    link_info.address_infos = [{
                        LINK_V4_ADDR_KEY: format_ipv4(subnet[1],30)
                    },
                    {
                        LINK_V4_ADDR_KEY: format_ipv4(subnet[2],30)
                    }]
                    cli.put_link(link_info)
                node_link_map[node_index][link_id] = link_info
                


        position_map: dict[str,Position] = {"":Position()}
        time_now = datetime.now()
        for instance_id,instance_info in all_instance_map.items():
            if instance_id in gs_updated_ids:
                p = Position()
                if instance_info.extra.get("role", "") == TU_ROLE:
                    p.latitude  = math.radians(tu_current_lat)
                    p.longitude = math.radians(tu_current_lon)
                else:
                    p.latitude  = float(instance_info.extra[EX_LATITUDE_KEY]) / 180 * math.pi
                    p.longitude = float(instance_info.extra[EX_LONGITUDE_KEY]) / 180 * math.pi
                p.altitude = float(instance_info.extra[EX_ALTITUDE_KEY])
                
                position_map[instance_id] = p
                continue
                
            if instance_info.start:
                new_postion = calculate_postion(instance_info,time_now)
                cli.put_position(instance_id,new_postion)
            else:
                new_postion = Position()
            position_map[instance_id] = new_postion
        # Do Ground Station Reconnect
    

        for ground_station in ground_station_list:
            if not ground_station.start:
                continue
            gs_position = position_map[ground_station.instance_id]
            satellite_id,change = select_closest_satellite(
                ground_station,
                position_map,
                all_instance_map
            )
            if change:
                address1 = {}
                address2 = {}
                
                old_link_id = ""
                for key in ground_station.connections.keys():
                    old_link_id = key
                    break
                if old_link_id != "":
                    old_link = cli.disable_link_between(
                        ground_station.node_index,
                        ground_station.instance_id,
                        ground_station.connections[key].end_node_index,
                        ground_station.connections[key].instance_id
                    )
                    logger.info("Switch %s from %s to %s"%(
                        ground_station.instance_id,
                        ground_station.connections[old_link_id].instance_id,
                        satellite_id
                    ))
                    old_sat_config = genenrate_config(cli,ground_station.connections[old_link_id].end_node_index,ground_station.connections[old_link_id].instance_id)
                    cli.put_instance_config(ground_station.connections[old_link_id].end_node_index,ground_station.connections[old_link_id].instance_id,json.dumps(old_sat_config))

                    # config_map = genenrate_config(
                    #     satellite_id,all_instance_map[ground_station.connections[key].instance_id],node_link_map)
                    if len(old_link) > 0:
                        address1 = old_link[old_link_id].address_infos[0]
                        address2 = old_link[old_link_id].address_infos[1]
                else:
                    subnet = alloc_ipv4(30)
                    address1 = {LINK_V4_ADDR_KEY:format_ipv4(subnet[1],30)}
                    address2 = {LINK_V4_ADDR_KEY:format_ipv4(subnet[2],30)}
                    logger.info("Switch %s from %s to %s"%(
                        ground_station.instance_id,
                        "None",
                        satellite_id
                    ))
                cli.enable_link_between(
                    ground_station.node_index,
                    ground_station.instance_id,
                    all_instance_map[satellite_id].node_index,
                    all_instance_map[satellite_id].instance_id,
                    address_info1=address1,
                    address_info2=address2,
                )
                gs_config = genenrate_config(cli,ground_station.node_index,ground_station.instance_id)
                # print(gs_config)
                cli.put_instance_config(ground_station.node_index,ground_station.instance_id,json.dumps(gs_config))
                sat_config = genenrate_config(cli,all_instance_map[satellite_id].node_index,all_instance_map[satellite_id].instance_id)
                # print(sat_config)
                cli.put_instance_config(all_instance_map[satellite_id].node_index,all_instance_map[satellite_id].instance_id,json.dumps(sat_config))

        

        for node_index,link_map in node_link_map.items():
            for link_id,link_info in link_map.items():
                if link_info.parameter is None:
                    link_info.parameter = {}
                if link_info.end_infos[0].instance_id=="" or link_info.end_infos[1].instance_id == "":
                    continue
                if not link_info.enable:
                    continue
                if link_info.end_infos[0].instance_type == TYPE_SATELLITE and \
                    link_info.end_infos[1].instance_type == TYPE_SATELLITE and \
                    all_instance_map[link_info.end_infos[1].instance_id].extra[EX_ORBIT_INDEX] != \
                    all_instance_map[link_info.end_infos[0].instance_id].extra[EX_ORBIT_INDEX] and \
                    (abs(position_map[link_info.end_infos[0].instance_id].latitude) > polar_threshold or \
                    abs(position_map[link_info.end_infos[1].instance_id].latitude) > polar_threshold):
                        # if PARAMETER_KEY_CONNECT in link_info.parameter.keys() and link_info.parameter[PARAMETER_KEY_CONNECT]==1:
                        #     logger.info("connect %s"%link_id)
                        link_info.parameter[PARAMETER_KEY_CONNECT] = 0
                else:
                    # if PARAMETER_KEY_CONNECT not in link_info.parameter.keys() or link_info.parameter[PARAMETER_KEY_CONNECT]==0:
                    #         logger.info("disconnect %s"%link_id)
                    link_info.parameter[PARAMETER_KEY_CONNECT] = 1


                if link_info.end_infos[0].instance_type == "" :
                    continue


                distance = distance_meter(
                    position_map[link_info.end_infos[0].instance_id],
                    position_map[link_info.end_infos[1].instance_id]
                )
                delay = int(get_propagation_delay_s(distance)*1000000)
                link_info.parameter[PARAMETER_KEY_DELAY] = delay
                link_info.parameter[PARAMETER_KEY_BANDWIDTH] = 1000000
                link_info.parameter[PARAMETER_KEY_LOSS] = 150
                cli.put_link_parameter(link_info.node_index,link_info.link_id,link_info.parameter)
                
        for instance_id,instance_info in all_instance_map.items():
            if not instance_info.start:
                continue
            config_map = genenrate_config(cli,instance_info.node_index,instance_id)
            cli.put_instance_config(instance_info.node_index,instance_id,json.dumps(config_map))
        sleep(step_second)
