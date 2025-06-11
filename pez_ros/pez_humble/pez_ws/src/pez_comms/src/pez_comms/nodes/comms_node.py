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


class CommsFullNode(Node):
    def __init__(self, config: Dict[str, Any]):
        super().__init__('comms_full')

        # --- 1) Pull out our ros__parameters dict ---
        ros_params = config.get('ros__parameters', config)

        # --- 2) Declare modem_io params with defaults ---
        modem_cfg = ros_params.get('modem_io', {})
        self.declare_parameter('modem_io.port',    modem_cfg.get('port', '/dev/ttyUSB0'))
        self.declare_parameter('modem_io.baud',    modem_cfg.get('baud', 9600))
        self.declare_parameter('modem_io.timeout', modem_cfg.get('timeout', 0.1))

        # --- 3) Declare any other flat scalars so get_parameter() works ---
        excluded = {'ros__parameters','modem_io','schedules','triggers',
                    'publishers','services','serial_handlers','plugins'}
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.declare_parameter(key, val)

        # --- 4) Provide a shutdown‐hook API for plugins ---
        self._on_shutdown_cbs = []
        self.add_on_shutdown = lambda cb: self._on_shutdown_cbs.append(cb)

        # --- 5) Override declared params with YAML values ---
        # modem_io overrides
        for subk, subv in modem_cfg.items():
            self.set_parameters([Parameter(f'modem_io.{subk}', value=subv)])
        # other scalars
        for key, val in ros_params.items():
            if key not in excluded and not isinstance(val, dict):
                self.set_parameters([Parameter(name=key, value=val)])

        # --- 6) Read final modem_io values ---
        port    = self.get_parameter('modem_io.port').value
        baud    = self.get_parameter('modem_io.baud').value
        timeout = self.get_parameter('modem_io.timeout').value

        # --- 7) Open your modem  ---
        self.modem = ModemIOMgr(port=port, baud=baud, timeout=timeout)
        self.get_logger().info(f"Modem opened on {port}@{baud}")


        # --- 8) Pull in our nested lists directly from the YAML ---
        schedules       = ros_params.get('schedules', [])
        triggers        = ros_params.get('triggers', [])
        publishers      = ros_params.get('publishers', [])
        services        = ros_params.get('services', [])
        serial_handlers = ros_params.get('serial_handlers', [])
        plugins         = ros_params.get('plugins', [])

        # --- 9) Wire up all subsystems ---
        self._serial_handlers = []
        self._setup_schedules(schedules)
        self._setup_triggers(triggers)
        self._setup_publishers(publishers)
        self._setup_services(services)
        self._setup_serial_handlers(serial_handlers)

        # --- 10) Load plugins (they can now call add_on_shutdown & modem.reopen) ---
        for plug in plugins:
            module = importlib.import_module(plug['module'])
            module.register(self, plug.get('config', {}))

        # --- 11) Start inbound thread if any handlers configured ---
        if serial_handlers:
            self._stop_event = threading.Event()
            self._serial_thread = threading.Thread(
                target=self._serial_loop, daemon=True
            )
            self._serial_thread.start()


    def _setup_serial_handlers(self, handlers):
        for h in handlers:
            pkt = get_packet(h['packet'])
            def matches(raw, pkt=pkt):
                try:
                    pkt.decode(raw)
                    return True
                except Exception:
                    return False

            pub_cfg = h.get('publish')
            pub = None
            if pub_cfg:
                mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
                cls = getattr(mod, pub_cfg['type'].split('/')[-1])
                pub = self.create_publisher(cls, pub_cfg['topic'], 10)

            self._serial_handlers.append({
                'matches':   matches,
                'pkt':       pkt,
                'decode':    h.get('decode', {}),
                'pub_cfg':   pub_cfg,
                'pub':       pub,
                'svc_map':   h.get('service_map', {}),
                'special':   h.get('special', {}),
            })

    def _serial_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            raw = self.modem.get_byte(timeout=0.1)
            if raw is None:
                continue
            for h in self._serial_handlers:
                if not h['matches'](raw):
                    continue
                fields = h['pkt'].decode(raw)
                sid = fields.get('service_id')
                if sid in h['svc_map']:
                    self._call_trigger(h['svc_map'][sid],
                                       seq=fields.get('seq', 0),
                                       value=fields.get('value', 0))
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
        if publisher:
            pub = publisher
            msg_cls = getattr(
                importlib.import_module(pub_cfg['type'].replace('/', '.')),
                pub_cfg['type'].split('/')[-1]
            )
        else:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            msg_cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            pub = self.create_publisher(msg_cls, pub_cfg['topic'], 10)

        msg = msg_cls()
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
                            kwargs: Dict[str, Any] = {}
                            for k, v in enc.items():
                                if k == 'type':
                                    continue
                                # resolve param:... or literal
                                raw = ( self.get_parameter(v.split(':',1)[1]).value
                                        if isinstance(v, str) and v.startswith('param:')
                                        else v )
                                # cast floats to ints for bitwise ops
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
                    st['name'], action,
                    duration=st.get('duration'),
                    wait_for=wait_for
                ))

            TransmissionScheduler(steps, loop=sched.get('loop', False)).start()

    def _setup_triggers(self, triggers):
        for trg in triggers:
            sub = trg['subscribe']
            mod = importlib.import_module(sub['type'].replace('/', '.'))
            cls = getattr(mod, sub['type'].split('/')[-1])
            def cb(msg, enc=trg['encode']):
                pd = get_packet(enc['type'])
                args: Dict[str, Any] = {}
                for k, v in enc.items():
                    if isinstance(v, str) and v.startswith('param:'):
                        args[k] = self.get_parameter(v.split(':',1)[1]).value
                    elif isinstance(v, str) and v.startswith('msg.'):
                        args[k] = getattr(msg, v.split('.',1)[1])
                    else:
                        args[k] = v
                self.modem.send_packet(pd.encode(**args))
            self.create_subscription(cls, sub['topic'], cb, sub.get('qos', 10))

    def _setup_publishers(self, pubs):
        for pub_cfg in pubs:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            def timer_cb(pub=pub_cfg, msg_cls=cls):
                self.create_publisher(msg_cls, pub['topic'], 10).publish(msg_cls())
            period = 1.0 / float(pub_cfg['rate'])
            self.create_timer(period, timer_cb)

    def _setup_services(self, svcs):
        for svc in svcs:
            mod = importlib.import_module(svc['srv_type'].replace('/', '.'))
            cls = getattr(mod, svc['srv_type'].split('/')[-1])
            def handle(req, resp, cfg=svc):
                enc = cfg['on_request']['encode']
                pd = get_packet(enc['type'])
                args = {
                    k: self.get_parameter(v.split(':',1)[1]).value
                    for k, v in enc.items()
                    if isinstance(v, str) and v.startswith('param:')
                }
                self.modem.send_packet(pd.encode(**args))
                self._do_publish(cfg['on_response']['publish'], {'resp': resp})
                resp.success = True
                resp.message = 'OK'
                return resp
            self.create_service(cls, svc['name'], handle)

    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        levels = (1 << bits) - 1
        return (q / levels) * 2.0 - 1.0

    def destroy_node(self):
        self.get_logger().info("Shutting down comms_full node.")
        # run plugin shutdown hooks
        for cb in self._on_shutdown_cbs:
            try:
                cb()
            except Exception:
                self.get_logger().error("Error in shutdown callback", exc_info=True)
        # stop serial thread
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
            self._serial_thread.join(timeout=1.0)
        try:
            self.modem.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config-file', required=True,
                        help='Path to YAML config file')
    args, _ = parser.parse_known_args()

    with open(args.config_file, 'r') as f:
        cfg = yaml.safe_load(f)

    rclpy.init(args=[])
    node = CommsFullNode(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
