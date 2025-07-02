#!/usr/bin/env python3
"""
comms_full_node_noetic.py
─────────────────────────
Generic, YAML-driven comms node for ROS-Noetic (rospy).

Key additions in this revision
──────────────────────────────
• Serial reader now maintains a 4-byte sliding window (`buf`) — each time
  it is full we try to decode **those four bytes** with every handler.

• Verbose logging:
    – DEBUG  : raw 4-byte window in hex  
    – DEBUG  : decode failures per packet class  
    – INFO   : successful decode with field dictionary  
    – INFO   : notice if **no** handler accepted the 4-byte word
"""

# ───── standard lib ────────────────────────────────────────────────────────
import argparse, importlib, threading, yaml
from typing import Any, Dict

# ───── ROS 1 ───────────────────────────────────────────────────────────────
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# ───── project libs ───────────────────────────────────────────────────────
from pez_comms.core.modem_io   import ModemIOMgr
from pez_comms.core.packet_def import get_packet
from pez_comms.core.scheduler  import TransmissionStep, TransmissionScheduler


# ───────────────────────────────────────────────────────────────────────────
# Helper: resolve "pkg/subdir/Class"  →  Python class
# ───────────────────────────────────────────────────────────────────────────
def _load_msg_or_srv(type_str: str):
    pkg, rest      = type_str.split('/', 1)     # e.g. geometry_msgs, msg/Twist
    subdir,  name  = rest.split('/', 1)         #          ↳      msg,  Twist
    module         = importlib.import_module(f"{pkg}.{subdir}")
    return getattr(module, name)


# ───────────────────────────────────────────────────────────────────────────
# Main class
# ───────────────────────────────────────────────────────────────────────────
class CommsFullNode:
    # ────────────────────────────────────────────────────────────────────
    def __init__(self, cfg: Dict[str, Any]):
        prm = cfg.get('ros__parameters', cfg)

        # — Modem —
        io = prm['modem_io']
        self.modem = ModemIOMgr(port=io['port'],
                                baud=io['baud'],
                                timeout=io.get('timeout', 0.1))
        rospy.loginfo("Modem @ %s %d baud (%.2fs timeout)",
                      io['port'], io['baud'], io.get('timeout', 0.1))

        # permit reopen from plugins
        setattr(self.modem, 'reopen',
                lambda p=io['port'], b=io['baud'], t=io.get('timeout',0.1):
                    ModemIOMgr(port=p, baud=b, timeout=t))

        # — Config lists —
        self._handlers = []
        self._setup_schedules(prm.get('schedules',        []))
        self._setup_triggers (prm.get('triggers',         []))
        self._setup_services (prm.get('services',         []))
        self._setup_handlers (prm.get('serial_handlers',  []))

        # — Start serial loop —
        self._stop = threading.Event()
        threading.Thread(target=self._serial_loop, daemon=True).start()
        rospy.loginfo("CommsFullNode ready")

    # ────────────────────────────────────────────────────────────────────
    # Serial handlers
    # ────────────────────────────────────────────────────────────────────
    def _setup_handlers(self, lst):
        for h in lst:
            pkt = get_packet(h['packet'])

            def match(raw, pkt=pkt):
                try:
                    pkt.decode(raw)
                    return True
                except Exception:
                    return False

            pub_cfg, pub = h.get('publish'), None
            if pub_cfg:
                cls = _load_msg_or_srv(pub_cfg['type'])
                pub = rospy.Publisher(pub_cfg['topic'], cls, queue_size=10)
                rospy.loginfo("Serial→topic: %s", pub_cfg['topic'])

            self._handlers.append(dict(
                match=match, pkt=pkt,
                pub_cfg=pub_cfg, pub=pub,
                svc_map=h.get('service_map', {})
            ))

    # serial read thread with 4-byte sliding window
    def _serial_loop(self):
        rospy.loginfo("Serial reader running (4-byte window)")
        buf = bytearray()
        while not rospy.is_shutdown() and not self._stop.is_set():
            b = self.modem.get_byte(timeout=0.1)
            if b is None:
                continue
            buf += b
            if len(buf) < 4:
                continue            # wait until we have 4 bytes
            if len(buf) > 4:        # keep only the newest 4
                buf = buf[-4:]

            raw4 = bytes(buf)
            rospy.logdebug("RX 4B window: %s", raw4.hex())

            handled = False
            for h in self._handlers:
                if not h['match'](raw4):
                    rospy.logdebug("%s decode failed",
                                   h['pkt'].__class__.__name__)
                    continue
                fields = h['pkt'].decode(raw4)
                rospy.loginfo("Decoded %s: %s",
                              h['pkt'].__class__.__name__, fields)

                sid = fields.get('service_id') or fields.get('svc_id')
                if sid in h['svc_map']:
                    self._call_trigger(h['svc_map'][sid])

                if h['pub_cfg'] and h['pub']:
                    self._publish_mapped(h['pub_cfg'], fields, h['pub'])
                handled = True
                break

            if not handled:
                rospy.logdebug("No handler accepted packet")

    # ────────────────────────────────────────────────────────────────
    # Triggers (topic → packet)
    # ────────────────────────────────────────────────────────────────
    def _setup_triggers(self, lst):
        for trg in lst:
            sub = trg['subscribe']
            cls = _load_msg_or_srv(sub['type'])

            def cb(msg, enc=trg['encode']):
                pkt_cls = get_packet(enc['type'])
                args = {}
                for k, v in enc.items():
                    if k == 'type':
                        continue
                    if isinstance(v, str) and v.startswith('msg.'):
                        ref = msg
                        for attr in v.split('.')[1:]:
                            ref = getattr(ref, attr)
                        args[k] = ref
                    else:
                        args[k] = v
                self.modem.send_packet(pkt_cls.encode(**args))

            rospy.Subscriber(sub['topic'], cls, cb,
                             queue_size=sub.get('qos', 10))
            rospy.loginfo("Trigger on %s → %s", sub['topic'], trg['encode']['type'])

    # ────────────────────────────────────────────────────────────────
    # Schedules
    # ────────────────────────────────────────────────────────────────
    def _setup_schedules(self, lst):
        for sch in lst:
            steps = []
            for st in sch.get('steps', []):
                action = None
                if 'encode' in st:
                    pkt_cls = get_packet(st['encode']['type'])
                    kwargs  = {k:v for k,v in st['encode'].items() if k!='type'}
                    action  = lambda p=pkt_cls, kw=kwargs: \
                              self.modem.send_packet(p.encode(**kw))
                steps.append(TransmissionStep(st['name'], action,
                                              duration=st.get('duration')))
            TransmissionScheduler(steps, loop=sch.get('loop', False)).start()
            rospy.loginfo("Schedule '%s' started", sch.get('name','<no-name>'))

    # ────────────────────────────────────────────────────────────────
    # Services  (service → packet)
    # ────────────────────────────────────────────────────────────────
    def _setup_services(self, lst):
        for svc in lst:
            srv_cls = _load_msg_or_srv(svc['srv_type'])
            enc     = svc['on_request']['encode']
            pkt_cls = get_packet(enc['type'])
            kwargs  = {k:v for k,v in enc.items() if k!='type'}

            def handle(_req, p=pkt_cls, kw=kwargs):
                self.modem.send_packet(p.encode(**kw))
                return TriggerResponse(True, "sent")

            rospy.Service(svc['name'], srv_cls, handle)
            rospy.loginfo("Service %s ready", svc['name'])

    # ────────────────────────────────────────────────────────────────
    # Helper: call mapped Trigger service
    # ────────────────────────────────────────────────────────────────
    def _call_trigger(self, srv_name: str):
        try:
            rospy.wait_for_service(srv_name, timeout=1.0)
            rospy.ServiceProxy(srv_name, Trigger)()
            rospy.loginfo("Service %s called by svc_map", srv_name)
        except Exception as e:
            rospy.logwarn("Service %s failed: %s", srv_name, e)

    # ────────────────────────────────────────────────────────────────
    # Helper: publish Twist (or other) with mapping
    # ────────────────────────────────────────────────────────────────
    def _publish_mapped(self, cfg, fields, pub):
        msg = pub.data_class()
        for path, key in cfg['mapping'].items():
            a, b = path.split('.')
            setattr(getattr(msg, a), b, fields[key])
        pub.publish(msg)


# ───────────────────────────────────────────────────────────────────────────
# Main
# ───────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-c','--config-file', required=True,
                    help='YAML config file')
    args = ap.parse_args(rospy.myargv()[1:])

    with open(args.config_file) as f:
        cfg = yaml.safe_load(f)

    rospy.init_node('comms_full')
    CommsFullNode(cfg)
    rospy.spin()


if __name__ == '__main__':
    main()
