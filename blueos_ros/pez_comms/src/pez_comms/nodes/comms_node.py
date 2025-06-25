#!/usr/bin/env python3
"""
comms_full_node_noetic.py

A single generic ROS Noetic (rospy) node that implements a fully configurable communications
engine driven by a YAML file. Supports:
  - Serial handlers (inbound packet decode → publish/service calls)
  - Scheduled packet transmissions
  - Topic‐triggered packet transmissions
  - Parameter-driven publishers
  - Service-based packet transmissions with optional follow-up publishes
  - Plugin modules for arbitrary custom logic (including modem reopen & shutdown hooks)
"""

import argparse
import yaml
import importlib
import threading
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from typing import Any, Dict

from pez_comms.core.modem_io import ModemIOMgr
from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler

class CommsFullNode:
    def __init__(self, config: Dict[str, Any]):
        # 1) Extract parameters from YAML
        ros_params = config.get('ros__parameters', config)
        rospy.loginfo("Initializing CommsFullNode with parameters from YAML")

        # 2) Modem configuration
        modem_cfg = ros_params.get('modem_io', {})
        port    = modem_cfg.get('port',    '/dev/ttyUSB0')
        baud    = modem_cfg.get('baud',     9600)
        timeout = modem_cfg.get('timeout',  0.1)
        rospy.loginfo(f"Modem settings: port={port}, baud={baud}, timeout={timeout}")

        # 3) Open modem and patch reopen()
        self.modem = ModemIOMgr(port=port, baud=baud, timeout=timeout)
        rospy.loginfo("Opened modem")
        def _reopen(p=port, b=baud, t=timeout):
            rospy.loginfo(f"Reopening modem on {p}@{b} (timeout={t})")
            try:
                self.modem.close()
            except Exception:
                pass
            self.modem = ModemIOMgr(port=p, baud=b, timeout=t)
            rospy.loginfo("Modem reopened")
        setattr(self.modem, 'reopen', _reopen)

        # 4) Shutdown hook
        rospy.on_shutdown(self.destroy_node)

        # 5) Extract all lists
        self._serial_handlers = []
        schedules       = ros_params.get('schedules', [])
        triggers        = ros_params.get('triggers', [])
        publishers      = ros_params.get('publishers', [])
        services        = ros_params.get('services', [])
        serial_handlers = ros_params.get('serial_handlers', [])
        plugins         = ros_params.get('plugins', [])

        # 6) Setup subsystems
        rospy.loginfo("Configuring schedules...")
        self._setup_schedules(schedules)
        rospy.loginfo("Configuring triggers...")
        self._setup_triggers(triggers)
        rospy.loginfo("Configuring publishers...")
        self._setup_publishers(publishers)
        rospy.loginfo("Configuring services...")
        self._setup_services(services)
        rospy.loginfo("Configuring serial handlers...")
        self._setup_serial_handlers(serial_handlers)

        # 7) Load plugins
        for plug in plugins:
            rospy.loginfo(f"Loading plugin {plug['module']}")
            module = importlib.import_module(plug['module'])
            module.register(self, plug.get('config', {}))
        rospy.loginfo("Plugins loaded")

        # 8) Start serial thread
        if self._serial_handlers:
            self._stop_event = threading.Event()
            self._serial_thread = threading.Thread(
                target=self._serial_loop, daemon=True
            )
            self._serial_thread.start()
            rospy.loginfo("Serial thread started")

    def _setup_serial_handlers(self, handlers):
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
                pub = rospy.Publisher(pub_cfg['topic'], cls, queue_size=10)
                rospy.loginfo(f"Serial → publisher on {pub_cfg['topic']}")

            self._serial_handlers.append({
                'matches': matches,
                'pkt':     pkt,
                'pub_cfg': pub_cfg,
                'pub':     pub,
                'svc_map': h.get('service_map', {}),
                'special': h.get('special', {}),
            })

    def _serial_loop(self):
        rospy.loginfo("Entering serial read loop")
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            raw = self.modem.get_byte(timeout=0.1)
            if raw is None:
                continue
            for h in self._serial_handlers:
                if not h['matches'](raw):
                    continue
                fields = h['pkt'].decode(raw)
                sid = fields.get('service_id')
                # 1) service_map
                if sid in h['svc_map']:
                    self._call_trigger(h['svc_map'][sid],
                                       seq=fields.get('seq', 0),
                                       value=fields.get('value', 0))
                # 2) special
                if sid in h['special']:
                    self._do_publish(h['special'][sid]['publish'], fields)
                # 3) normal publish
                if h['pub_cfg'] and h['pub']:
                    self._do_publish(h['pub_cfg'], fields, publisher=h['pub'])
                break

    def _call_trigger(self, srv_name: str, seq: int=0, value: Any=0):
        rospy.loginfo(f"Calling Trigger service: {srv_name}")
        try:
            rospy.wait_for_service(srv_name, timeout=2.0)
            proxy = rospy.ServiceProxy(srv_name, Trigger)
            resp = proxy()
            ack = resp.success
        except Exception as e:
            rospy.logwarn(f"Service {srv_name} failed: {e}")
            ack = False

        pktB = get_packet('B_COMMAND')
        resp_seq = seq if ack else (1 - seq)
        self.modem.send_packet(
            pktB.encode(service_id=0, value=value, seq=resp_seq)
        )
        rospy.loginfo(f"Sent {'ACK' if ack else 'NACK'} seq={resp_seq}")

    def _do_publish(self, pub_cfg, fields, publisher=None):
        topic = pub_cfg['topic']
        if publisher:
            pub = publisher
            msg_cls = getattr(
                importlib.import_module(pub_cfg['type'].replace('/', '.')),
                pub_cfg['type'].split('/')[-1]
            )
        else:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            msg_cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            pub = rospy.Publisher(topic, msg_cls, queue_size=10)

        msg = msg_cls()
        # mapping fields → message
        for path, expr in pub_cfg.get('mapping', {}).items():
            val = eval(expr, {'__builtins__': {}}, {**fields, 'dequant': self._dequant})
            parts = path.split('.')
            setattr(getattr(msg, parts[0]), parts[1], val)
        # single‐field expr
        if 'expr' in pub_cfg:
            val = eval(pub_cfg['expr'], {'__builtins__': {}}, fields)
            setattr(msg, pub_cfg['type'].split('/')[-1].lower(), val)

        pub.publish(msg)
        rospy.loginfo(f"Published on {topic}")

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
                                raw = enc[k]
                                if isinstance(raw, str) and raw.startswith('param:'):
                                    key = raw.split(':', 1)[1]
                                    raw = config_params.get(key, raw)
                                if isinstance(raw, float):
                                    raw = int(raw)
                                kwargs[k] = raw
                            self.modem.send_packet(pd.encode(**kwargs))
                        return act
                    action = make_act(st['encode'])

                wait_for = None
                if 'wait_for_param' in st:
                    key = st['wait_for_param']
                    wait_for = lambda key=key: bool(config_params.get(key))

                steps.append(
                    TransmissionStep(
                        st['name'], action,
                        duration=st.get('duration'),
                        wait_for=wait_for
                    )
                )

            TransmissionScheduler(steps, loop=sched.get('loop', False)).start()
            rospy.loginfo(f"Started schedule '{sched.get('name','')}'")

    def _setup_triggers(self, triggers):
        for trg in triggers:
            sub = trg['subscribe']
            mod = importlib.import_module(sub['type'].replace('/', '.'))
            cls = getattr(mod, sub['type'].split('/')[-1])

            def cb(msg, enc=trg['encode']):
                pd = get_packet(enc['type'])
                args = {}
                for k, v in enc.items():
                    if isinstance(v, str) and v.startswith('param:'):
                        key = v.split(':', 1)[1]
                        args[k] = config_params.get(key)
                    elif isinstance(v, str) and v.startswith('msg.'):
                        args[k] = getattr(msg, v.split('.',1)[1])
                    else:
                        args[k] = v
                self.modem.send_packet(pd.encode(**args))

            rospy.Subscriber(sub['topic'], cls, cb, queue_size=sub.get('qos', 10))
            rospy.loginfo(f"Trigger subscribed to {sub['topic']}")

    def _setup_publishers(self, pubs):
        for pub_cfg in pubs:
            mod = importlib.import_module(pub_cfg['type'].replace('/', '.'))
            cls = getattr(mod, pub_cfg['type'].split('/')[-1])
            period = rospy.Duration(1.0 / float(pub_cfg['rate']))

            def timer_cb(event, pub_cfg=pub_cfg, msg_cls=cls):
                pub = rospy.Publisher(pub_cfg['topic'], msg_cls, queue_size=10)
                pub.publish(msg_cls())

            rospy.Timer(period, timer_cb)
            rospy.loginfo(f"Heartbeat publisher on {pub_cfg['topic']} at {pub_cfg['rate']} Hz")

    def _setup_services(self, svcs):
        for svc in svcs:
            mod = importlib.import_module(svc['srv_type'].replace('/', '.'))
            srv_cls = getattr(mod, svc['srv_type'].split('/')[-1])
            resp_cfg = svc['on_response']['publish']
            enc_cfg  = svc['on_request']['encode']

            def handle(req, enc=enc_cfg, resp_cfg=resp_cfg):
                pd = get_packet(enc['type'])
                args = {
                    k: config_params[v.split(':',1)[1]]
                    for k, v in enc.items() if isinstance(v, str) and v.startswith('param:')
                }
                self.modem.send_packet(pd.encode(**args))
                self._do_publish(resp_cfg, {'resp': None})
                return TriggerResponse(success=True, message='OK')

            rospy.Service(svc['name'], srv_cls, handle)
            rospy.loginfo(f"Service {svc['name']} ready")

    @staticmethod
    def _dequant(q: int, bits: int) -> float:
        sign_bit = 1 << (bits - 1)
        if q & sign_bit:
            q -= (1 << bits)
        max_mag = (1 << (bits - 1)) - 1
        return q / max_mag

    def destroy_node(self):
        rospy.loginfo("Shutting down CommsFullNode...")
        try:
            self._stop_event.set()
            self._serial_thread.join(timeout=1.0)
        except AttributeError:
            pass
        try:
            self.modem.close()
            rospy.loginfo("Modem closed")
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config-file', required=True,
                        help='Path to YAML config file')
    args = parser.parse_args(rospy.myargv()[1:])

    # 1) Load YAML
    with open(args.config_file, 'r') as f:
        cfg = yaml.safe_load(f)
    rospy.loginfo(f"Loaded config from {args.config_file}")

    # 2) Init ROS node
    rospy.init_node('comms_full', anonymous=False)

    # 3) Make config_params global for easy access in closures
    global config_params
    config_params = cfg.get('ros__parameters', cfg)

    # 4) Instantiate
    CommsFullNode(cfg)

    # 5) Spin
    rospy.spin()

if __name__ == '__main__':
    main()
