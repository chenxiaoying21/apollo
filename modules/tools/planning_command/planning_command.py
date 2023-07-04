import modules.common_msgs.planning_msgs.planning_command_pb2 as planning_command_pb2
import modules.common_msgs.external_command_msgs.free_space_command_pb2 as free_space_command_pb2
import modules.common_msgs.routing_msgs.routing_pb2 as routing_pb2
import modules.common_msgs.external_command_msgs.command_status_pb2 as command_status_pb2
import modules.common_msgs.external_command_msgs.action_command_pb2 as action_pb2
import google.protobuf.any_pb2 as any_pb2
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
import modules.tools.common.proto_utils as proto_utils
seq_num = 1
polygon = [(752700.83, 2566002),
           (752700, 2566016),
           (752688, 2566016),
           (752689, 2566002)]
pose = {}
pose['1'] = [752691, 2566014, 3.14]
pose['2'] = [752697, 2566005, 0]
pose['3'] = [752699, 2566015, -1.57]
pose['4'] = [752691, 2566007, 1.57]


def add_header(planning_command):
    global seq_num
    planning_command.header.sequence_num = seq_num
    planning_command.header.timestamp_sec = cyber_time.Time.now().to_sec()
    planning_command.header.module_name = "planning_command"
    seq_num = seq_num + 1


if __name__ == '__main__':
    cyber.init()
    node = cyber.Node("planning_command")
    writer = node.create_writer(
        "/apollo/planning_command", planning_command_pb2.PlanningCommand)
    client_free_space = node.create_client(
        "/apollo/external_command/free_space", free_space_command_pb2.FreeSpaceCommand, command_status_pb2.CommandStatus)
    client_pull_over = node.create_client(
        "/apollo/external_command/action", action_pb2.ActionCommand, command_status_pb2.CommandStatus)
    while not cyber.is_shutdown():
        m = input("0:lanefollow 1~4: free space 9:pullover\n")
        planning_command = planning_command_pb2.PlanningCommand()
        add_header(planning_command)
        print(m)
        if m == '0':
            act = action_pb2.ActionCommand()
            add_header(act)
            act.command = action_pb2.ActionCommandType.FOLLOW
            client_pull_over.send_request(act)
        elif m == "9":
            act = action_pb2.ActionCommand()
            add_header(act)
            act.command = action_pb2.ActionCommandType.PULL_OVER
            client_pull_over.send_request(act)

        else :
            fsc = free_space_command_pb2.FreeSpaceCommand()
            add_header(fsc)
            fsc.parking_spot_pose.x = pose[m][0]
            fsc.parking_spot_pose.y = pose[m][1]
            fsc.parking_spot_pose.heading = pose[m][2]
            roi = fsc.parking_roi.add()
            for pt in polygon:
                pt_pb = roi.point.add()
                pt_pb.x = pt[0]
                pt_pb.y = pt[1]
            fsc.target_speed = 3.0
            client_free_space.send_request(fsc)
            print(fsc)
            # msg = any_pb2.Any()
            # msg.Pack(fsc, str(fsc))
            # planning_command.custom_command.CopyFrom(msg)
            # writer.write(planning_command)

    cyber.shutdown()
