#!/usr/bin/env python3
"""
comms_node.py

A single generic ROS2 node that implements a fully configurable communications
engine driven by a YAML file. Supports:
  - Serial handlers (inbound packet decode → publish/topic/service)
  - Scheduled packet transmissions
  - Topic-triggered packet transmissions
  - Parameter-driven publishers
  - Service-based packet transmissions with optional follow-up publishes
  - Plugin modules for arbitrary custom logic
"""

import yaml
import importlib
import threading
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from pez_comms.core.modem_io import ModemIOMgr
from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler
from std_srvs.srv import Trigger


class CommsFullNode(Node):
    def __init__(self):
        super().__init__('comms_full')

        # 1) Load config
        self.declare_parameter('config', '')
        cfg_path = self.get_parameter('config').value
        cfg = yaml.safe_load(open(cfg_path))

        # 2) Modem I/O manager
        m = cfg.get('modem_io', {})
        port    = m.get('port', '/dev/ttyUSB0')
        baud    = m.get('baud', 9600)
        timeout = m.get('timeout', 0.1)
        self.modem = ModemIOMgr(port=port, baud=baud, timeout=timeout)
        self.get_logger().info(f"Modem opened on {port}@{baud}")

        # 3) Declare ROS parameters
        for name, default in cfg.get('parameters', {}).items():
            self.declare_parameter(name, default)

        # 4) Setup each section
        self._serial_handlers: list = []
        self._setup_serial_handlers(cfg.get('serial_handlers', []))

        self._setup_schedules(cfg.get('schedules', []))
        self._setup_triggers(cfg.get('triggers', []))
        self._setup_publishers(cfg.get('publishers', []))
        self._setup_services(cfg.get('services', []))

        # 5) Load plugins
        for plug in cfg.get('plugins', []):
            module = importlib.import_module(plug['module'])
            module.register(self, plug['config'])

        # 6) Start serial‐read thread
        self._stop_event = threading.Event()
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()

    def _setup_serial_handlers(self, handlers):
        """
        handlers: list of dicts with keys:
          - packet: packet_id string
          - decode: mapping field_name→decoded_key
          - publish (optional): {topic, type, mapping: {msg_field: expr}}
          - service_map (optional): {svc_id: service_name, ...}
          - special (optional): {svc_id: {publish: {...}}}
        """
        for h in handlers:
            pkt = get_packet(h['packet'])
            def matches(raw, pkt=pkt):
                try:
                    pkt.decode(raw)
                    return True
                except Exception:
                    return False

            decode_map = h.get('decode', {})
            pub_cfg    = h.get('publish', None)
            svc_map    = h.get('service_map', {})
            special    = h.get('special', {})

            # create publisher if needed
            pub = None
            if pub_cfg:
                # import message class
                mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
                cls = getattr(mod, pub_cfg['type'].split('/')[-1])
                pub = self.create_publisher(cls, pub_cfg['topic'], 10)

            self._serial_handlers.append({
                'matches': matches,
                'pkt':      pkt,
                'decode':  decode_map,
                'pub_cfg': pub_cfg,
                'pub':     pub,
                'svc_map': svc_map,
                'special': special
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
                # 1) service_map handling
                sid = fields.get('service_id')
                if sid is not None and sid in h['svc_map']:
                    srv_name = h['svc_map'][sid]
                    self._call_trigger(srv_name, seq=fields.get('seq', 0), value=fields.get('value',0))
                # 2) special handling
                if 'special' in h and sid in h['special']:
                    pubspec = h['special'][sid]['publish']
                    self._do_publish(pubspec, fields)
                # 3) generic publish
                if h['pub_cfg'] and h['pub']:
                    self._do_publish(h['pub_cfg'], fields, publisher=h['pub'])
                break

    def _call_trigger(self, srv_name: str, seq:int=0, value:Any=0):
        cli = self.create_client(Trigger, srv_name)
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service {srv_name} unavailable")
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        ack = fut.done() and fut.result().success
        # send ACK/NACK back
        # assume B_COMMAND packet registered:
        pktB = get_packet('B_COMMAND')
        resp_seq = seq if ack else (1 - seq)
        self.modem.send_packet(pktB.encode(service_id=0, value=value, seq=resp_seq))

    def _do_publish(self, pub_cfg, fields, publisher=None):
        """
        pub_cfg: dict with keys 'topic','type','mapping' or 'expr'
        fields: decoded fields dict
        publisher: optional pre-created ROS publisher
        """
        # import or use existing publisher
        if publisher:
            pub = publisher
            msg_cls = pub_cfg and importlib.import_module(pub_cfg['type'].replace('/','.')).__getattribute__(
                pub_cfg['type'].split('/')[-1]
            )
        else:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            msg_cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            pub = self.create_publisher(msg_cls, pub_cfg['topic'], 10)

        msg = msg_cls()
        # mapping: nested paths → Python expressions
        for path, expr in pub_cfg.get('mapping', {}).items():
            val = eval(expr, {'__builtins__':{}}, {**fields, 'dequant': self._dequant})
            parts = path.split('.')
            attr = getattr(msg, parts[0])
            setattr(attr, parts[1], val)
        # or single expr
        if 'expr' in pub_cfg:
            val = eval(pub_cfg['expr'], {'__builtins__':{}}, {**fields})
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
                        return lambda: self.modem.send_packet(
                            pd.encode(**{
                                k: ( self.get_parameter(v.split(':',1)[1]).value
                                     if isinstance(v,str) and v.startswith('param:')
                                     else v )
                                for k,v in enc.items() if k!='type'
                            })
                        )
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
            TransmissionScheduler(steps, loop=sched.get('loop',False)).start()

    def _setup_triggers(self, triggers):
        for trg in triggers:
            sub = trg['subscribe']
            msg_mod = importlib.import_module(sub['type'].replace('/','.'))
            cls = getattr(msg_mod, sub['type'].split('/')[-1])
            def cb(msg, enc=trg['encode']):
                pd = get_packet(enc['type'])
                args = {}
                for k,v in enc.items():
                    if isinstance(v,str) and v.startswith('param:'):
                        args[k] = self.get_parameter(v.split(':',1)[1]).value
                    elif isinstance(v,str) and v.startswith('msg.'):
                        args[k] = getattr(msg, v.split('.',1)[1])
                    else:
                        args[k] = v
                self.modem.send_packet(pd.encode(**args))
            self.create_subscription(cls, sub['topic'], cb, sub.get('qos',10))

    def _setup_publishers(self, pubs):
        for pub_cfg in pubs:
            mod = importlib.import_module(pub_cfg['type'].replace('/','.'))
            cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            def timer_cb(pub=pub_cfg, msg_cls=cls):
                msg = msg_cls()
                # no fields: user only wants heartbeat etc.
                self.create_publisher(msg_cls,pub['topic'],10).publish(msg)
            period = 1.0 / float(pub['rate'])
            self.create_timer(period, timer_cb)

    def _setup_services(self, svcs):
        for svc in svcs:
            mod = importlib.import_module(svc['srv_type'].replace('/','.'))
            cls = getattr(mod, svc['srv_type'].split('/')[-1])
            def handle(req, resp, cfg=svc):
                # on_request.encode
                enc = cfg['on_request']['encode']
                pd = get_packet(enc['type'])
                args = {
                    k: self.get_parameter(v.split(':',1)[1]).value
                    for k,v in enc.items() if isinstance(v,str) and v.startswith('param:')
                }
                self.modem.send_packet(pd.encode(**args))
                # on_response.publish
                pub_cfg = cfg['on_response']['publish']
                self._do_publish(pub_cfg, {'resp': resp})
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
        self._stop_event.set()
        try:
            self.modem.close()
        except:
            pass
        self._serial_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommsFullNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
