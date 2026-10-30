"""Linux SDK reader; publishes intent only, never robot targets or authority."""
import json
import threading
import time
from pathlib import Path
from .reader import ReportStream
from .mapping import BUTTONS


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import Joy,Imu
    from morla_runtime.resource_manager import ResourceManager
    rclpy.init(args=args)

    class Driver(Node):
        def __init__(self):
            super().__init__('vader5_input')
            for k,v in {'device':'','publish_hz':50.,'read_timeout_s':.1,'fake':False,
                        'topic':'/vader5/events'}.items():self.declare_parameter(k,v)
            self.done=threading.Event();self.locks=ResourceManager();self.pad=None
            self.device=self.get_parameter('device').value;self.fake=self.get_parameter('fake').value
            hz=self.get_parameter('publish_hz').value;self.timeout=self.get_parameter('read_timeout_s').value
            self.stream=ReportStream(publish_hz=hz,backend='fake' if self.fake else 'real')
            qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
            self.events=self.create_publisher(String,self.get_parameter('topic').value,qos)
            self.joy=self.create_publisher(Joy,'~/joy',10);self.imu=self.create_publisher(Imu,'~/imu',10)
            self.status=self.create_publisher(String,'~/status',10)
            self.worker=threading.Thread(target=self.run,name='vader5-sdk',daemon=True);self.worker.start()

        def emit(self,row):
            self.events.publish(String(data=json.dumps(row,allow_nan=False)))
            if not row['connected']:return
            joy=Joy();joy.header.stamp=self.get_clock().now().to_msg();joy.header.frame_id='vader5_device'
            joy.axes=row['sticks']+row['triggers'];joy.buttons=[int(b in row['buttons']) for b in BUTTONS]
            self.joy.publish(joy)
            imu=Imu();imu.header=joy.header;imu.orientation_covariance[0]=-1.
            imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=row['gyro']
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=row['accel']
            self.imu.publish(imu)

        def run(self):
            try:
                if not self.fake:
                    if not self.device:raise ValueError('Explicit /dev/hidrawN or stable symlink required')
                    path=Path(self.device).resolve(strict=True)
                    if path.parent!=Path('/dev') or not path.name.startswith('hidraw'):
                        raise ValueError('Select a hidraw vendor interface')
                    self.locks.acquire(['vader5:transport:'+str(path)])
                    from vader5_sdk import Vader5
                    self.pad=Vader5(str(path));self.pad.open()
                while not self.done.is_set():
                    if self.fake:
                        row=self.stream.packet();self.emit(row) if row is not None else None
                        self.done.wait(self.stream.period/1e9);continue
                    batch=self.pad.read_batch(timeout=self.timeout)
                    if len(batch)>128:raise RuntimeError('Vader input backlog exceeds bounded batch')
                    for index,state in enumerate(batch):
                        if self.done.is_set():break
                        row=self.stream.packet(state,allow_tick=index==len(batch)-1)
                        if row is not None:self.emit(row)
            except Exception as exc:
                self.get_logger().error(str(exc))
                self.status.publish(String(data=json.dumps({'connected':False,'error':str(exc)})))
            finally:
                try:self.emit(self.stream.packet(connected=False))
                finally:
                    try:
                        if self.pad is not None:self.pad.close()
                    finally:self.locks.release()

        def close(self):
            self.done.set();self.worker.join(timeout=2)
            if self.worker.is_alive():raise TimeoutError('Vader SDK reader did not close')

    node=None
    try:
        node=Driver();rclpy.spin(node)
    finally:
        if node is not None:
            try:node.close()
            finally:node.destroy_node()
        rclpy.shutdown()
