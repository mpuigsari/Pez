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
from pez_comms.core.modem_io import ModemIOMgr
from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler
from std_srvs.srv import Trigger


def main():
    # 1) Parse only our --config-file flag, leave ROS args in ros_args
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '-c', '--config-file',
        required=True,
        help='Path to YAML config file'
    )
    args, ros_args = parser.parse_known_args()

    # 2) Logger for startup
    startup_logger = rclpy.logging.get_logger('comms_full')
    startup_logger.info(f"Loading config file: {args.config_file}")

    # 3) Load YAML config
    with open(args.config_file, 'r') as f:
        cfg = yaml.safe_load(f)
    startup_logger.info("Config file loaded")

    # 4) Init ROS with remappings
    rclpy.init(args=ros_args)
    startup_logger.info(f"rclpy initialized with remappings: {ros_args}")

    # 5) Instantiate node
    node = CommsFullNode(cfg)
    startup_logger.info("CommsFullNode instantiated, entering spin")

    # 6) Spin
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

        # --- 1) Extract ROS parameters dict ---
        ros_params = config.get('ros__parameters', config)
        self.get_logger().debug(f"ros__parameters: {ros_params}")

        # --- 2) Declare modem_io parameters ---
        modem_cfg = ros_params.get('modem_io', {})
        self.declare_parameter('modem_io.port',    modem_cfg.get('port',    '/dev/ttyUSB0'))
        self.declare_parameter('modem_io.baud',    modem_cfg.get('baud',     9600))
        self.declare_parameter('modem_io.timeout', modem_cfg.get('timeout',  0.1))
        self.get_logger().info(f"Declared modem_io params: {modem_cfg}")

        # --- 3) Declare other flat parameters ---
        excluded = {'ros__parameters','modem_io','schedules','triggers',
                    'publishers','services','serial_handlers','plugins'}
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.declare_parameter(key, val)
                self.get_logger().debug(f"Declared param: {key}={val}")

        # --- 4) Shutdown hooks API ---
        self._on_shutdown_cbs = []
        self.add_on_shutdown = lambda cb: self._on_shutdown_cbs.append(cb)
        self.get_logger().info("Shutdown hook API ready")

        # --- 5) Override parameters with YAML values ---
        for subk, subv in modem_cfg.items():
            self.set_parameters([Parameter(f'modem_io.{subk}', value=subv)])
        self.get_logger().debug(f"Set modem_io params: {modem_cfg}")
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.set_parameters([Parameter(name=key, value=val)])
                self.get_logger().debug(f"Set param: {key}={val}")

        # --- 6) Read back final modem settings ---
        port    = self.get_parameter('modem_io.port').value
        baud    = self.get_parameter('modem_io.baud').value
        timeout = self.get_parameter('modem_io.timeout').value
        self.get_logger().info(f"Final modem settings: port={port}, baud={baud}, timeout={timeout}")

        # --- 7) Open modem and patch reopen ---
        self.modem = ModemIOMgr(port=port, baud=baud, timeout=timeout)
        self.get_logger().info(f"Modem opened on {port}@{baud}")
        # Add reopen for plugins
        def _reopen(p, b, t):
            self.get_logger().info(f"Reopening modem on {p}@{b} (timeout={t})")
            try:
                self.modem.close()
            except Exception:
                pass
            self.modem = ModemIOMgr(port=p, baud=b, timeout=t)
            self.get_logger().info(f"Modem reopened on {p}@{b}")
        setattr(self.modem, 'reopen', _reopen)

        # --- 8) Extract config lists ---
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

        # --- 9) Setup each subsystem ---
        self.get_logger().info("Setting up schedules...")
        self._setup_schedules(schedules)
        self.get_logger().info("Setting up triggers...")
        self._setup_triggers(triggers)
        self.get_logger().info("Setting up publishers...")
        self._setup_publishers(publishers)
        self.get_logger().info("Setting up services...")
        self._setup_services(services)
        self.get_logger().info("Setting up serial handlers...")
        self._setup_serial_handlers(serial_handlers)

        # --- 10) Load plugins ---
        for plug in plugins:
            self.get_logger().info(f"Loading plugin: {plug['module']}")
            module = importlib.import_module(plug['module'])
            module.register(self, plug.get('config', {}))
        self.get_logger().info("Plugins loaded")

        # --- 11) Start serial thread if enabled ---
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
                mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
                cls = getattr(mod, pub_cfg['type'].split('/')[-1])
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
                    self.get_logger().info(f"Triggering service {h['svc_map'][sid]}")
                    self._call_trigger(h['svc_map'][sid], seq=fields.get('seq',0), value=fields.get('value',0))
                if sid in h['special']:
                    self.get_logger().info(f"Handling special for id={sid}")
                    self._do_publish(h['special'][sid]['publish'], fields)
                if h['pub_cfg'] and h['pub']:
                    self.get_logger().info(f"Publishing serial data on {h['pub_cfg']['topic']}")
                    self._do_publish(h['pub_cfg'], fields, publisher=h['pub'])
                break

    def _call_trigger(self, srv_name: str, seq: int = 0, value: Any = 0):
        self.get_logger().info(f"Calling trigger srv={srv_name}")
        cli = self.create_client(Trigger, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service {srv_name} unavailable")
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        ack = fut.done() and fut.result().success
        pktB = get_packet('B_COMMAND')
        resp_seq = seq if ack else (1-seq)
        self.modem.send_packet(pktB.encode(service_id=0, value=value, seq=resp_seq))
        self.get_logger().info(f"Sent {'ACK' if ack else 'NACK'} seq={resp_seq}")

    def _do_publish(self, pub_cfg, fields, publisher=None):
        topic = pub_cfg['topic']
        self.get_logger().debug(f"Publishing via {topic} with mapping {pub_cfg.get('mapping')}")
        if publisher:
            pub = publisher
            msg_cls = getattr(importlib.import_module(pub_cfg['type'].replace('/', '.')), pub_cfg['type'].split('/')[-1])
        else:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            msg_cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            pub = self.create_publisher(msg_cls, topic, 10)
        msg = msg_cls()
        for path, expr in pub_cfg.get('mapping', {}).items():
            val = eval(expr, {'__builtins__':{}}, {**fields, 'dequant': self._dequant})
            parts = path.split('.')
            setattr(getattr(msg, parts[0]), parts[1], val)
        if 'expr' in pub_cfg:
            val = eval(pub_cfg['expr'], {'__builtins__':{}}, {**fields})
            setattr(msg, pub_cfg['type'].split('/')[-1].lower(), val)
        pub.publish(msg)
        self.get_logger().info(f"Published on {topic}")

    def _setup_schedules(self, scheds):
        for sched in scheds:
            self.get_logger().info(f"Configuring schedule '{sched.get('name','')}' loop={sched.get('loop')}")
            steps = []
            for st in sched.get('steps', []):
                action = None
                if 'encode' in st:
                    pd = get_packet(st['encode']['type'])
                    def make_act(enc):
                        def act():
                            kwargs: Dict[str, Any] = {}
                            for k,v in enc.items():
                                if k=='type': continue
                                raw = (self.get_parameter(v.split(':',1)[1]).value
                                       if isinstance(v,str) and v.startswith('param:') else v)
                                if isinstance(raw,float): raw=int(raw)
                                kwargs[k] = raw
                            self.modem.send_packet(pd.encode(**kwargs))
                            self.get_logger().debug(f"Scheduled send {pd} with {kwargs}")
                        return act
                    action = make_act(st['encode'])
                wait_for = None
                if 'wait_for_param' in st:
                    key = st['wait_for_param']
                    wait_for = lambda key=key: bool(self.get_parameter(key).value)
                steps.append(TransmissionStep(st['name'], action, duration=st.get('duration'), wait_for=wait_for))
            TransmissionScheduler(steps, loop=sched.get('loop',False)).start()
            self.get_logger().info(f"Started schedule '{sched.get('name','')}'")

    def _setup_triggers(self, triggers):
        for trg in triggers:
            sub = trg['subscribe']
            self.get_logger().info(f"Setting up trigger on {sub['topic']}")
            mod = importlib.import_module(sub['type'].replace('/','.'))
            cls = getattr(mod, sub['type'].split('/')[-1])
            def cb(msg, enc=trg['encode']):
                self.get_logger().info(f"Trigger callback on {sub['topic']} with msg {msg}")
                pd = get_packet(enc['type'])
                args={}                
                for k,v in enc.items():
                    if isinstance(v,str) and v.startswith('param:'):
                        args[k]=self.get_parameter(v.split(':',1)[1]).value
                    elif isinstance(v,str) and v.startswith('msg.'):
                        args[k]=getattr(msg, v.split('.',1)[1])
                    else:
                        args[k]=v
                self.modem.send_packet(pd.encode(**args))
                self.get_logger().info(f"Published triggered packet {pd} with args {args}")
            self.create_subscription(cls, sub['topic'], cb, sub.get('qos',10))

    def _setup_publishers(self, pubs):
        for pub_cfg in pubs:
            self.get_logger().info(f"Setting up heartbeat publisher on {pub_cfg['topic']} at {pub_cfg['rate']}Hz")
            mod = importlib.import_module(pub_cfg['type'].replace('/','.'))
            cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            def timer_cb(pub=pub_cfg, msg_cls=cls):
                msg=msg_cls(); self.create_publisher(msg_cls,pub['topic'],10).publish(msg);
                self.get_logger().debug(f"Published heartbeat on {pub['topic']}")
            period=1.0/float(pub_cfg['rate'])
            self.create_timer(period, timer_cb)

    def _setup_services(self, svcs):
        for svc in svcs:
            self.get_logger().info(f"Setting up service {svc['name']} of type {svc['srv_type']}")
            mod=importlib.import_module(svc['srv_type'].replace('/','.'))
            cls=getattr(mod, svc['srv_type'].split('/')[-1])
            def handle(req,resp,cfg=svc):
                self.get_logger().info(f"Service {cfg['name']} called")
                enc=cfg['on_request']['encode']; pd=get_packet(enc['type'])
                args={k:self.get_parameter(v.split(':',1)[1]).value for k,v in enc.items() if isinstance(v,str) and v.startswith('param:')}
                self.modem.send_packet(pd.encode(**args))
                self.get_logger().debug(f"Sent service packet {pd} with args {args}")
                self._do_publish(cfg['on_response']['publish'], {'resp':resp})
                resp.success=True; resp.message='OK'
                return resp
            self.create_service(cls, svc['name'], handle)

    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        sign_bit = 1 << (bits - 1)
        if q & sign_bit:
            q -= (1 << bits)
        max_mag = (1 << (bits - 1)) - 1
        return q / max_mag

    def destroy_node(self):
        self.get_logger().info("Destroying CommsFullNode, running cleanup...")
        for cb in self._on_shutdown_cbs:
            try:
                cb()
                self.get_logger().debug("Ran shutdown callback")
            except Exception:
                self.get_logger().error("Error in shutdown callback", exc_info=True)
        if hasattr(self,'_stop_event'):
            self._stop_event.set(); self._serial_thread.join(timeout=1.0)
            self.get_logger().info("Serial thread joined")
        try:
            self.modem.close()
            self.get_logger().info("Modem closed")
        except Exception:
            pass
        super().destroy_node()
