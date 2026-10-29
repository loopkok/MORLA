"""ROS input adapters; the device drivers are independent of robot models.

Quest sources subscribe to the guarded morla_input outputs, never raw network
packets. The gamepad source expects joy/game_controller_node's SDL layout.
"""
from __future__ import annotations
import threading
import time
from dataclasses import replace
from .adapters.normalized import InputStream,GamepadAdapter,InputFrame,InputError


class RosInputManager:
    def __init__(self,instances,on_frame,on_disconnect,*,clock=time.monotonic_ns):
        self.instances=instances; self.on_frame=on_frame; self.on_disconnect=on_disconnect
        self.clock=clock; self.sources={}; self.cache={}; self.node=None

    def start(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import PoseStamped,PoseArray
        from std_msgs.msg import String
        if not rclpy.ok(): rclpy.init()
        self.node=Node('morla_composed_inputs_'+str(id(self)))
        self.executor=SingleThreadedExecutor(); self.executor.add_node(self.node)
        for instance in self.instances:
            iid=instance['instance_id']; plugin=instance['plugin_id']
            if plugin=='morla.input.quest_right':
                import json
                from .quest_right import QuestRightSource
                source=QuestRightSource(iid, clock=self.clock,
                    on_transition=lambda iid=iid:self.on_disconnect(iid))
                self.sources[iid]=source
                topic=instance.get('parameters',{}).get('input_topic','/quest3/right_teleop_events')
                def event(msg,iid=iid):
                    def action():
                        if len(msg.data)>16384:raise InputError('Quest event oversized')
                        row=json.loads(msg.data)
                        source=self.sources[iid]
                        frame=source.event(row)
                        if source.control_event(row,frame):self.on_frame(frame)
                    self._guarded(iid,action)
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                event_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,depth=16)
                self.node.create_subscription(String,topic,event,event_qos)
            elif 'gamepad' in plugin or 'xbox' in plugin:
                source=GamepadAdapter(iid,clock=self.clock); source.reconnect(); self.sources[iid]=source
                topic=instance.get('parameters',{}).get('input_topic','/joy')
                self.node.create_subscription(Joy,topic,lambda msg,iid=iid:self._joy(iid,msg),qos_profile_sensor_data)
            elif plugin in {'morla.input.quest','morla.input.glove'}:
                source=InputStream(iid,clock=self.clock); source.reconnect(); self.sources[iid]=source
                self.cache[iid]={}
                if plugin=='morla.input.quest':
                    self.node.create_subscription(String,'/quest3/input_epoch',lambda msg,iid=iid:self._epoch(iid),10)
                    for side in ('left','right'):
                        self.node.create_subscription(PoseStamped,f'/quest3/{side}_wrist_pose',lambda msg,iid=iid,side=side:self._pose(iid,side,msg),qos_profile_sensor_data)
                        self.node.create_subscription(Joy,f'/quest3/{side}_controller_joy',lambda msg,iid=iid,side=side:self._quest_joy(iid,side,msg),qos_profile_sensor_data)
                for side in ('left','right'):
                    self.node.create_subscription(PoseArray,f'/hand_landmarks/{side}',lambda msg,iid=iid,side=side:self._landmarks(iid,side,msg),qos_profile_sensor_data)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True); self.thread.start()

    def _guarded(self,iid,fn):
        try: fn()
        except Exception as exc:
            self.on_disconnect(iid)
            self.node.get_logger().error(f'input {iid} rejected: {exc}')

    def _joy(self,iid,message):
        def action():
            source=self.sources[iid]
            if source.latest is not None and source.read() is None: source.reconnect()
            self.on_frame(source.joy(list(message.axes),list(message.buttons)))
        self._guarded(iid,action)

    def _epoch(self,iid):
        self.sources[iid].reconnect(); self.cache[iid]={}; self.on_disconnect(iid)

    def _emit(self,iid,values,frame_id):
        source=self.sources[iid]; now=self.clock()
        # Keep separate per-port ages; a fresh button cannot refresh an old pose.
        for key,value in values.items(): self.cache[iid][key]=(value,now,frame_id)
        current={key:value for key,(value,stamp,_) in self.cache[iid].items() if now-stamp<=250_000_000}
        frame=source.accept(current,source.sequence+1,epoch=source.epoch,frame_id=frame_id)
        metadata={key:{'received_ns':stamp,'frame_id':fid} for key,(_,stamp,fid) in self.cache[iid].items() if key in current}
        frame=replace(frame,port_metadata=metadata); source.latest=frame
        self.on_frame(frame)

    def _pose(self,iid,side,msg):
        def action():
            p,q=msg.pose.position,msg.pose.orientation
            self._emit(iid,{side+'_pose':{'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w]}},msg.header.frame_id)
        self._guarded(iid,action)

    def _quest_joy(self,iid,side,msg):
        def action():
            if len(msg.axes)<4: raise InputError('Quest Joy shape mismatch')
            values={side+'_grip':msg.axes[1]>.5,'scalar':float(msg.axes[0]),
                    'axes':{'x':float(msg.axes[2]),'y':float(msg.axes[3])}}
            self._emit(iid,values,msg.header.frame_id or 'input')
        self._guarded(iid,action)

    def _landmarks(self,iid,side,msg):
        def action():
            if len(msg.poses)!=21: raise InputError('landmark topology must contain 21 points')
            points=[[p.position.x,p.position.y,p.position.z] for p in msg.poses]
            self._emit(iid,{side+'_landmarks':points},msg.header.frame_id)
        self._guarded(iid,action)

    def close(self):
        if self.node is None: return
        self.executor.shutdown(timeout_sec=1)
        self.thread.join(timeout=1)
        self.node.destroy_node(); self.node=None
