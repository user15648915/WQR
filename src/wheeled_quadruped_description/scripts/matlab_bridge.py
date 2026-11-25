import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import socket, threading, json

JOINTS = [
    'fl_hip_joint', 'fl_thigh_joint', 'fl_shank_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_shank_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_shank_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_shank_joint'
]

class MatlabBridge(Node):
    def __init__(self):
        super().__init__('matlab_bridge')
        self.pub_traj = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        
        self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/detected_obstacle', self.obs_cb, 10)
        
        self.state = {"position": []}
        self.obs = {"distance": 99.0, "height": 0.0}
        
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('0.0.0.0', 5005))
        self.srv.listen(5)
        threading.Thread(target=self.serve_clients, daemon=True).start()

    def state_cb(self, m):
        self.state["position"] = list(m.position)

    def obs_cb(self, m):
        if len(m.data) >= 2:
            self.obs = {"distance": round(float(m.data[0]), 3), "height": round(float(m.data[1]), 3)}

    def serve_clients(self):
        while rclpy.ok():
            c, _ = self.srv.accept()
            threading.Thread(target=self.handle_client, args=(c,), daemon=True).start()

    def handle_client(self, c):
        buf = ""
        try:
            while rclpy.ok():
                data = c.recv(4096).decode('utf-8')
                if not data: break
                buf += data
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line: continue
                    
                    if line == "GET_OBSTACLE":
                        c.sendall((json.dumps(self.obs) + "\n").encode())
                    elif line == "GET_STATE":
                        c.sendall((json.dumps(self.state) + "\n").encode())
                    else:
                        d = json.loads(line)
                        if "wheel_command" in d:
                            w = Float64MultiArray()
                            w.data = [float(x) for x in d["wheel_command"]]
                            self.pub_wheel.publish(w)
                        if "joint_command" in d:
                            jt = JointTrajectory()
                            jt.joint_names = JOINTS
                            # Пустой header.stamp означает "выполнить мгновенно без привязки к часам"
                            
                            p = JointTrajectoryPoint()
                            p.positions = [float(x) for x in d["joint_command"]]
                            p.time_from_start = Duration(sec=0, nanosec=200000000) # 0.2 секунды на достижение точки
                            jt.points = [p]
                            
                            self.pub_traj.publish(jt)
        except Exception: pass
        finally: c.close()

def main(args=None):
    rclpy.init(args=args)
    node = MatlabBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
