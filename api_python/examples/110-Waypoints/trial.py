import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2


# Create Sequences:
    # Sequence 1 - Pick and Place Object 1, and so on
    # Each sequence will require 3 Actions and 2 Gripper commands
    # Action 1 can be a default position of the arm
    # Action 2 can be picking position of the object
    # Action 3 can be placing position of the object
    # Gripper command 1 can be opening the gripper
    # Gripper command 2 can be closing the gripper



TIMEOUT_DURATION = 30
# Create closure to set an event after an END or an ABORT

def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def check_for_sequence_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications on a sequence

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e = e):
        event_id = notification.event_identifier
        task_id = notification.task_index
        if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
            print("Sequence task {} completed".format(task_id))
        elif event_id == Base_pb2.SEQUENCE_ABORTED:
            print("Sequence aborted with error {}:{}"\
                .format(\
                    notification.abort_details,\
                    Base_pb2.SubErrorCodes.Name(notification.abort_details)))
            e.set()
        elif event_id == Base_pb2.SEQUENCE_COMPLETED:
            print("Sequence completed.")
            e.set()
    return check


# For user information:
def action_types_list(base, base_cyclic):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    print("Setting Servoing Mode to Single Level Servoing")
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Print the list of available actions
    print("Printing the list of available actions")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    for action in action_list.action_list:
        print("Action name: {}, handle: {}".format(action.name, action.handle))


def packaging_mode(base):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    print("Setting Servoing Mode to Single Level Servoing")
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to Packaging Position
    print("Moving the arm to packaging position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Packaging":
            action_handle = action.handle
    
    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished 

def vertical_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    print("Setting Servoing Mode to Single Level Servoing")
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    print("Moving the arm to Vertical position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Zero":
            action_handle = action.handle
    
    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished 

def grip_open(base):

    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = 0.0
    gripper_command.mode = Base_pb2.GRIPPER_POSITION

    print('Opening the gripper...')
    base.SendGripperCommand(gripper_command)
    time.sleep(3)

def grip_close(base):

    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = 1.0
    gripper_command.mode = Base_pb2.GRIPPER_POSITION

    print('Closing the gripper...')
    base.SendGripperCommand(gripper_command)
    time.sleep(3)



def create_action(base, base_cyclic):
    
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    print("Setting Servoing Mode to Single Level Servoing")
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Define a new Action
    print("Defining a new Action")
    new_action = Base_pb2.Action()
    new_action.name = "Test"
    new_action.application_data = ""

   
    feedback = base_cyclic.RefreshFeedback()


    cartesian_pose = new_action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x - 0.1       # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.5    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.3    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)

    return new_action

def populateCartesianCoordinate(waypointInformation):
    
    waypoint = Base_pb2.CartesianWaypoint()  
    waypoint.pose.x = waypointInformation[0]
    waypoint.pose.y = waypointInformation[1]
    waypoint.pose.z = waypointInformation[2]
    waypoint.blending_radius = waypointInformation[3]
    waypoint.pose.theta_x = waypointInformation[4]
    waypoint.pose.theta_y = waypointInformation[5]
    waypoint.pose.theta_z = waypointInformation[6] 
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    
    return waypoint

def create_sequence(base, base_cyclic):

    print("Creating actions for sequence")
    action1 = create_action(base, base_cyclic)
   
    # Create a new sequence
    print("Creating a new sequence")
    new_sequence = Base_pb2.Sequence()
    new_sequence.name = "Test sequence"

    # Add the actions to the sequence
    print("Adding the actions to the sequence")
    task1 = new_sequence.tasks.add()
    task1.group_identifier = 0
    task1.action.CopyFrom(action1)

    e = threading.Event()
    notification_handle = base.OnNotificationSequenceInfoTopic(
        check_for_sequence_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing the sequence")
    handle_sequence = base.CreateSequence(new_sequence)
    base.PlaySequence(handle_sequence)

    # Leave time to sequence to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on sequence notification wait")
    return finished

def main():

    # Import the utilities helper module

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        #gripper = GripperCommand(router)

        success = action_types_list(base, base_cyclic)

        # #Uncomment the actions you want to perform
        success = packaging_mode(base)
        success = vertical_position(base)
        # success = create_sequence(base, base_cyclic)


        # Functions that work
        success = grip_open(base)
        success = grip_close(base)

    
if __name__ == "__main__":
    main()

