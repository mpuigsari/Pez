import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from pez_comms.core.packet_def import get_packet


def register(node: Node, cfg: dict):
    """
    Host‐side plugin: either
      • AB mode: continuous PacketA @ rate + PacketB state-machine
      • 40 mode: 1 Hz Packet40 + pause/unpause around PacketB ACK

    cfg keys:
      mode           : 'AB' or '40'
      packet_a       : str
      packet_b       : str
      packet_40      : str
      topic_cmd_vel  : str
      topic_camera   : str
      services       : dict {svc_id: name}
      camera_svc_id  : int
      broadcast_rate : float
      ack_timeout    : float
      svc_wait       : float
    """

    mode    = cfg.get('mode','AB').upper()
    pA      = get_packet(cfg['packet_a'])
    pB      = get_packet(cfg['packet_b'])
    p40     = get_packet(cfg.get('packet_40',''))
    rate    = float(cfg.get('broadcast_rate',4.0))
    ack_to  = float(cfg.get('ack_timeout',2.0))
    sw_wait = float(cfg.get('svc_wait',5.0))
    cam_id  = int(cfg['camera_svc_id'])

    # state
    axes         = {'x':0.0,'y':0.0,'z':0.0}
    svc_cmd      = None       # (sid,val)
    pending_sent = False
    last_seq     = 0
    ack_ready    = threading.Event()
    ack_success  = False

    # helpers
    def quant(v,bits):
        lvl = (1<<bits)-1
        return max(0,min(lvl,int(round((v+1.0)/2.0*lvl))))

    # subs / services setup
    node.create_subscription(Twist, cfg['topic_cmd_vel'],
        lambda m: axes.update(x=m.linear.x, y=m.linear.y, z=m.linear.z),
        10)

    def cam_cb(msg: Float32):
        nonlocal svc_cmd, pending_sent
        if svc_cmd:  return
        svc_cmd = (cam_id, 1 if msg.data>0 else 0)
        pending_sent = False
    node.create_subscription(Float32, cfg['topic_camera'], cam_cb, 10)

    for sid_s,name in cfg['services'].items():
        sid = int(sid_s)
        if sid==cam_id: continue
        # Trigger services
        def mk(s,v):
            def handler(req,res):
                nonlocal svc_cmd,pending_sent
                if svc_cmd:
                    res.success=False; res.message="busy"
                    return res
                svc_cmd = (s,v)
                pending_sent = False
                # wait for ack
                start=time.time()
                while rclpy.ok() and not ack_ready.is_set() and time.time()-start<ack_to:
                    time.sleep(0.01)
                res.success = ack_success
                res.message = "ok" if res.success else "nack"
                return res
            return handler
        if sid==0:
            node.create_service(Trigger,f"{name}_start",mk(0,1))
            node.create_service(Trigger,f"{name}_stop", mk(0,0))
        else:
            node.create_service(Trigger,name, mk(sid,1))

    # incoming PacketB handler (ACK/NACK)
    def on_b(raw: bytes):
        nonlocal ack_success
        f = pB.decode(raw)
        ack_success = (f['seq']==last_seq)
        ack_ready.set()
    node.register_serial_handler(pB.packet_id, on_b)


    stop_evt = threading.Event()

    def loop_ab():
        slot = 1.0/rate
        while not stop_evt.is_set():
            if not svc_cmd:
                x_q = quant(axes['x'],3)
                y_q = quant(axes['y'],2)
                z_q = quant(axes['z'],2)
                node.modem.send_packet(pA.encode(x=x_q,y=y_q,z=z_q))
                time.sleep(slot)
            else:
                # send PacketB
                sid,val  = svc_cmd
                last_seq  = last_seq^1
                node.modem.send_packet(pB.encode(service_id=sid,value=val,seq=last_seq))
                # wait ack
                ack_ready.clear()
                start = time.time()
                while rclpy.ok() and not ack_ready.is_set() and time.time()-start<sw_wait:
                    time.sleep(0.01)
                svc_cmd = None

    def loop_40():
        while not stop_evt.is_set():
            if svc_cmd and not pending_sent:
                sid,val   = svc_cmd
                node.modem.send_packet(p40.encode(
                    seq=last_seq,
                    vx=quant(axes['x'],8),
                    vy=quant(axes['y'],8),
                    vz=quant(axes['z'],8),
                    svc_pending=1,
                    svc_id=sid,
                    svc_val=val
                ))
                pending_sent = True
                # wait ack
                ack_ready.clear()
                start=time.time()
                while rclpy.ok() and not ack_ready.is_set() and time.time()-start<sw_wait:
                    time.sleep(0.01)
                svc_cmd = None
                last_seq ^= 1
            else:
                node.modem.send_packet(p40.encode(
                    seq=last_seq,
                    vx=quant(axes['x'],8),
                    vy=quant(axes['y'],8),
                    vz=quant(axes['z'],8),
                    svc_pending=0, svc_id=0, svc_val=0
                ))
                last_seq = (last_seq+1)&0x0F
                time.sleep(1.0)

    # start chosen loop
    if mode=='AB':
        t = threading.Thread(target=loop_ab, daemon=True)
    elif mode=='40':
        t = threading.Thread(target=loop_40, daemon=True)
    else:
        raise RuntimeError(f"host_side: bad mode '{mode}'")

    t.start()
    node.add_on_shutdown(lambda: stop_evt.set())
