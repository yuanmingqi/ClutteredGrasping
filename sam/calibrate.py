import rtde_receive
import rtde_control
import rtde_io

class RTDE_ROBOT:
    def __init__(self,
                 ip_address,
                 standby_pose=[0.2835031437979863, -0.3077173607169412, 0.39, 
                               -2.156284915564007, -2.240367221971588, -0.014276005449558397]
                 ) -> None:
        self.ip_address = ip_address
        self.standby_pose = standby_pose

        # RTDE interface
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip_address)
        self.rtde_c = rtde_control.RTDEControlInterface(self.ip_address)
        self.rtde_io = rtde_io.RTDEIOInterface(self.ip_address)

        # Set the standby pose
        self.rtde_c.moveL(self.standby_pose, speed=0.05)


if __name__ == "__main__":
    robot = RTDE_ROBOT("169.254.159.50")
    tcp_pose = robot.rtde_r.getActualTCPPose()
    print(tcp_pose)

    # real sense=0.63m
    # ratio 50pixels = 0.05m
    # minimum height = 0.25523473156483356m
    # (640, 480) [80:, 160:610] (400, 450, 3)

    dx_pixels, dy_pixels = 450 - 150, 162
    dx_meters, dy_meters = dx_pixels / 50, dy_pixels / 50

    slave_pose = tcp_pose
    slave_pose[0] -= (dx_meters * 0.05 + 0.05)
    slave_pose[1] -= (dy_meters * 0.05 + 0.05)
    print(slave_pose)

    # robot.rtde_c.moveL(slave_pose, speed=0.1)