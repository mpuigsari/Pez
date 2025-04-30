#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# internal state (could hook into your controller)
magnet_on = False

def handle_start(req):
    rospy.loginfo("[Service] Start swimming")
    # TODO: put your “start” logic here
    return TriggerResponse(success=True, message="Swim started")

def handle_stop(req):
    rospy.loginfo("[Service] Stop swimming")
    # TODO: put your “stop” logic here
    return TriggerResponse(success=True, message="Swim stopped")

def handle_toggle_magnet(req):
    global magnet_on
    magnet_on = not magnet_on
    rospy.loginfo(f"[Service] Magnet {'ON' if magnet_on else 'OFF'}")
    # TODO: actually set hardware here
    return TriggerResponse(success=True,
                           message=f"Magnet {'ON' if magnet_on else 'OFF'}")

if __name__ == "__main__":
    rospy.init_node("pez_services")
    s1 = rospy.Service("teleoperation/start_swim", Trigger, handle_start)
    s2 = rospy.Service("teleoperation/stop_swim",  Trigger, handle_stop)
    s3 = rospy.Service("teleoperation/toggle_magnet", Trigger, handle_toggle_magnet)
    rospy.loginfo("Pez services ready: start_swim, stop_swim, toggle_magnet")
    rospy.spin()
