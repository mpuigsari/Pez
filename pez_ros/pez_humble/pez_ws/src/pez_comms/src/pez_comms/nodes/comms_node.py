#!/usr/bin/env python3
"""
comms_full_node.py

A single generic ROS2 node that implements a fully configurable communications
engine driven by a YAML file (loaded via PyYAML). Supports:
  - Serial handlers (inbound packet decode → publish/topic/service)
  - Scheduled packet transmissions
  - Topic-triggered packet transmissions
  - Parameter-driven publishers
  - Service-based packet transmissions with optional follow-up publishes
  - Plugin modules for arbitrary custom logic (including modem reopen & shutdown hooks)
"""

import argparse
import yaml
import importlib
import threading
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from pez_comms.core.modem_io import ModemIOMgr
from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler


def _load_msg_or_srv_class(type_str: str):
    """
    Given a ROS 2 type string like "geometry_msgs/msg/Twist" or
    "std_srvs/srv/Trigger", return the corresponding Python class.
    """
    pkg, rest = type_str.split('/', 1)         # e.g. "geometry_msgs", "msg/Twist"
    subdir, name = rest.split('/', 1)          # e.g. "msg", "Twist"
    module_name = f"{pkg}.{subdir}"            # e.g. "geometry_msgs.msg"
    module = importlib.import_module(module_name)
    return getattr(module, name)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '-c', '--config-file',
        required=True,
        help='Path to YAML config file'
    )
    args, ros_args = parser.parse_known_args()

    startup_logger = rclpy.logging.get_logger('comms_full')
    startup_logger.info(f"Loading config file: {args.config_file}")

    with open(args.config_file, 'r') as f:
        cfg = yaml.safe_load(f)
    startup_logger.info("Config file loaded")

    rclpy.init(args=ros_args)
    startup_logger.info(f"rclpy initialized with remappings: {ros_args}")

    node = CommsFullNode(cfg)
    startup_logger.info("CommsFullNode instantiated, entering spin")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        startup_logger.info("KeyboardInterrupt caught, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


class CommsFullNode(Node):
    def __init__(self, config: Dict[str, Any]):
        super().__init__('comms_full')
        self.get_logger().info("Initializing CommsFullNode...")

        ros_params = config.get('ros__parameters', config)
        self.get_logger().debug(f"ros__parameters: {ros_params}")

        modem_cfg = ros_params.get('modem_io', {})
        self.declare_parameter('modem_io.port',    modem_cfg.get('port',    '/dev/ttyUSB0'))
        self.declare_parameter('modem_io.baud',    modem_cfg.get('baud',     9600))
        self.declare_parameter('modem_io.timeout', modem_cfg.get('timeout',  0.1))
        self.get_logger().info(f"Declared modem_io params: {modem_cfg}")

        excluded = {'ros__parameters','modem_io','schedules','triggers',
                    'publishers','services','serial_handlers','plugins'}
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.declare_parameter(key, val)
                self.get_logger().debug(f"Declared param: {key}={val}")

        self._on_shutdown_cbs = []
        self.add_on_shutdown = lambda cb: self._on_shutdown_cbs.append(cb)
        self.get_logger().info("Shutdown hook API ready")

        for subk, subv in modem_cfg.items():
            self.set_parameters([Parameter(f'modem_io.{subk}', value=subv)])
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.set_parameters([Parameter(name=key, value=val)])

        port    = self.get_parameter('modem_io.port').value
        baud    = self.get_parameter('modem_io.baud').value
        timeout = self.get_parameter('modem_io.timeout').value
        self.get_logger().info(f"Final modem settings: port={port}, baud={baud}, timeout={timeout}")

        self.modem = ModemIOMgr(port=port, baud=baud, timeout=timeout)
        self.get_logger().info(f"Modem opened on {port}@{baud}")
        def _reopen(p, b, t):
            self.get_logger().info(f"Reopening modem on {p}@{b} (timeout={t})")
            try:
                self.modem.close()
            except Exception:
                pass
            self.modem = ModemIOMgr(port=p, baud=b, timeout=t)
            self.get_logger().info(f"Modem reopened on {p}@{b}")
        setattr(self.modem, 'reopen', _reopen)

        schedules       = ros_params.get('schedules', [])
        triggers        = ros_params.get('triggers', [])
        publishers      = ros_params.get('publishers', [])
        services        = ros_params.get('services', [])
        serial_handlers = ros_params.get('serial_handlers', [])
        plugins         = ros_params.get('plugins', [])
        self.get_logger().info(
            f"Config lists - schedules:{len(schedules)}, triggers:{len(triggers)}, "
            f"publishers:{len(publishers)}, services:{len(services)}, "
            f"serial_handlers:{len(serial_handlers)}, plugins:{len(plugins)}"
        )

        self._setup_schedules(schedules)
        self._setup_triggers(triggers)
        self._setup_publishers(publishers)
        self._setup_services(services)
        self._setup_serial_handlers(serial_handlers)

        for plug in plugins:
            self.get_logger().info(f"Loading plugin: {plug['module']}")
            module = importlib.import_module(plug['module'])
            module.register(self, plug.get('config', {}))
        self.get_logger().info("Plugins loaded")

        if serial_handlers:
            self._stop_event = threading.Event()
            self._serial_thread = threading.Thread(
                target=self._serial_loop, daemon=True
            )
            self._serial_thread.start()
            self.get_logger().info("Serial loop thread started")

        self.get_logger().info("CommsFullNode initialization complete")

    def _setup_serial_handlers(self, handlers):
        self.get_logger().debug(f"Configuring {len(handlers)} serial handlers")
        self._serial_handlers = []
        for h in handlers:
            pkt = get_packet(h['packet'])
            def matches(raw, pkt=pkt):
                try:
                    pkt.decode(raw)
                    return True
                except:
                    return False

            pub_cfg = h.get('publish')
            pub = None
            if pub_cfg:
                cls = _load_msg_or_srv_class(pub_cfg['type'])
                pub = self.create_publisher(cls, pub_cfg['topic'], 10)
                self.get_logger().info(f"Created serial publisher on {pub_cfg['topic']}")

            self._serial_handlers.append({
                'matches': matches,
                'pkt':     pkt,
                'decode':  h.get('decode', {}),
                'pub_cfg': pub_cfg,
                'pub':     pub,
                'svc_map': h.get('service_map', {}),
                'special': h.get('special', {}),
            })
        self.get_logger().info("Serial handlers configured")

    def _serial_loop(self):
        self.get_logger().info("Entering serial loop")
        while rclpy.ok() and not self._stop_event.is_set():
            raw = self.modem.get_byte(timeout=0.1)
            if raw is None:
                continue
            self.get_logger().debug(f"Serial raw: {raw}")
            for h in self._serial_handlers:
                if not h['matches'](raw):
                    continue
                fields = h['pkt'].decode(raw)
                sid = fields.get('service_id')
                self.get_logger().info(f"Decoded packet id={sid}")
                if sid in h['svc_map']:
                    self._call_trigger(
                        h['svc_map'][sid],
                        seq=fields.get('seq', 0),
                        value=fields.get('value', 0)
                    )
                if sid in h['special']:
                    self._do_publish(h['special'][sid]['publish'], fields)
                if h['pub_cfg'] and h['pub']:
                    self._do_publish(h['pub_cfg'], fields, publisher=h['pub'])
                break

    def _call_trigger(self, srv_name: str, seq: int = 0, value: Any = 0):
        cli = self.create_client(Trigger, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service {srv_name} unavailable")
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        ack = fut.done() and fut.result().success
        pktB = get_packet('B_COMMAND')
        resp_seq = seq if ack else (1 - seq)
        self.modem.send_packet(
            pktB.encode(service_id=0, value=value, seq=resp_seq)
        )

    def _do_publish(self, pub_cfg, fields, publisher=None):
        cls = _load_msg_or_srv_class(pub_cfg['type'])
        if publisher:
            pub = publisher
        else:
            pub = self.create_publisher(cls, pub_cfg['topic'], 10)

        msg = cls()
        for path, expr in pub_cfg.get('mapping', {}).items():
            val = eval(expr, {'__builtins__': {}}, {**fields, 'dequant': self._dequant})
            parts = path.split('.')
            setattr(getattr(msg, parts[0]), parts[1], val)
        if 'expr' in pub_cfg:
            val = eval(pub_cfg['expr'], {'__builtins__': {}}, {**fields})
            setattr(msg, pub_cfg['type'].split('/')[-1].lower(), val)

        pub.publish(msg)

    def _setup_schedules(self, scheds):
        for sched in scheds:
            steps = []
            for st in sched.get('steps', []):
                action = None
                if 'encode' in st:
                    pd = get_packet(st['encode']['type'])
                    def make_act(enc):
                        def act():
                            kwargs = {}
                            for k, v in enc.items():
                                if k == 'type':
                                    continue
                                if isinstance(v, str) and v.startswith('param:'):
                                    raw = self.get_parameter(v.split(':', 1)[1]).value
                                else:
                                    raw = v
                                if isinstance(raw, float):
                                    raw = int(raw)
                                kwargs[k] = raw
                            self.modem.send_packet(pd.encode(**kwargs))
                        return act
                    action = make_act(st['encode'])
                wait_for = None
                if 'wait_for_param' in st:
                    key = st['wait_for_param']
                    wait_for = lambda key=key: bool(self.get_parameter(key).value)
                steps.append(TransmissionStep(
                    st['name'],
                    action,
                    duration=st.get('duration'),
                    wait_for=wait_for
                ))
            TransmissionScheduler(steps, loop=sched.get('loop', False)).start()

    # ------------------------------------------------------------------
    # 4)  TOPIC-TRIGGERED PACKETS  (e.g. /bluerov/cmd_vel)
    # ------------------------------------------------------------------
    def _setup_triggers(self, triggers):
        for trg in triggers:
            sub = trg['subscribe']
            cls = _load_msg_or_srv_class(sub['type'])

            def cb(msg, enc=trg['encode']):
                pd   = get_packet(enc['type'])
                args = {}
                for k, v in enc.items():
                    if k == 'type':                 # skip meta-key
                        continue
                    if isinstance(v, str) and v.startswith('param:'):
                        args[k] = self.get_parameter(v.split(':', 1)[1]).value
                    elif isinstance(v, str) and v.startswith('msg.'):
                        obj = msg
                        for attr in v.split('.')[1:]:
                            obj = getattr(obj, attr)
                        args[k] = obj
                    else:
                        args[k] = v

                raw = pd.encode(**args)                                       # NEW
                self.get_logger().info(f"TX trigger {pd.packet_id}: "          # NEW
                                    f"{raw.hex()}  ← {args}")              # NEW
                self.modem.send_packet(raw)

            self.create_subscription(cls, sub['topic'], cb, sub.get('qos', 10))


    def _setup_publishers(self, pubs):
        for pub_cfg in pubs:
            cls = _load_msg_or_srv_class(pub_cfg['type'])
            def timer_cb(pub=pub_cfg, msg_cls=cls):
                msg = msg_cls()
                self.create_publisher(msg_cls, pub['topic'], 10).publish(msg)
            period = 1.0 / float(pub_cfg['rate'])
            self.create_timer(period, timer_cb)

    # ------------------------------------------------------------------
    # 5)  SERVICE-BASED PACKET SENDERS  (/start, /stop, /lights …)
    # ------------------------------------------------------------------
    def _setup_services(self, svcs):
        for svc in svcs:
            srv_cls = _load_msg_or_srv_class(svc['srv_type'])

            def handle(req, resp, cfg=svc):
                enc = cfg['on_request']['encode']
                pd  = get_packet(enc['type'])

                kwargs = {k: (self.get_parameter(v.split(':', 1)[1]).value
                            if isinstance(v, str) and v.startswith('param:')
                            else v)
                        for k, v in enc.items() if k != 'type'}

                raw = pd.encode(**kwargs)                                     # NEW
                self.get_logger().info(f"TX service {cfg['name']} "
                                    f"{pd.packet_id}: {raw.hex()} ← {kwargs}") # NEW
                self.modem.send_packet(raw)

                resp.success, resp.message = True, 'OK'
                return resp

            self.create_service(srv_cls, svc['name'], handle)


    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        sign_bit = 1 << (bits - 1)
        if q & sign_bit:
            q -= (1 << bits)
        max_mag = (1 << (bits - 1)) - 1
        return q / max_mag

    def destroy_node(self):
        for cb in self._on_shutdown_cbs:
            try:
                cb()
            except Exception:
                self.get_logger().error("Error in shutdown callback", exc_info=True)
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
            self._serial_thread.join(timeout=1.0)
        try:
            self.modem.close()
        except Exception:
            pass
        super().destroy_node()


if __name__ == '__main__':
    main()
