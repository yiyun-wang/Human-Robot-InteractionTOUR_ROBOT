from std_msgs.msg import String                 # Used to process ROS images
from geometry_msgs.msg import Twist             # Sends velocity commands to the robot
import playsound                                # Play .mp3 file
from gtts import gTTS                           # Text-to-speech
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

class TourRobot(Node):
    '''
    Node class that controls the robot, 
    stores relevant variables for the state machine, 
    social navigation, and text-to-speech during HRI.
    '''
    def __init__(self, user_position, dest_position, robot_expression):
        super().__init__('tour_robot_node')

        # Set initial parameters
        self.user_position = user_position
        self.dest_positions = dest_position
        self.robot_expression = robot_expression

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            qos_profile_sensor_data)
        
        # Create a publisher which can "talk" to robot and tell it to move
        self.movement_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            1)

        # Create a Twist message and add linear x and angular z values
        self.move_cmd = Twist()

        # Initialize navigation object
        self.navigator = TurtleBot4Navigator()
        
        # Set initial pose
        self.initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(self.initial_pose)

        # Initialize robot poses
        self.goal_pose = []

        # Define states for state machine
        self.state1 = 'approach_user'
        self.state2 = 'give_tour'
        self.state3 = 'depart_user'
        self.state4 = 'end'

        # State Machine Variables
        self.curr_state = self.state1 # track current state
        self.next_state = self.state1 # track next state

    def update_state_machine(self):
        """Add Comments
        -------
        """
        if(self.curr_state == self.state1):
            # Wait for Nav2
            self.navigator.waitUntilNav2Active()
            
            # Set goal poses
            self.goal_pose = []
            # self.add_goal(self.userdatetime A combination of a date and a time. Attributes: 
            self.add_goal(self.user_position['x'], 
                          self.user_position['y'],
                          self.user_position['direction'])
            
            # Robot alerts user that it is apporaching user

            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)
            
            # Robot greets user and ask for their destination
            self.robot_talker(robot_phrase='Hello, my name is Turtlbot. I will escort you to your destination.')

            # Generate robot expression
            if(self.robot_expression == 'gesture'):
                self.generate_gesture_expression()
            elif(self.robot_expression == 'lightring'):
                self.generate_lightring_expression()
            else: 
                self.generate_gesture_lightring_expression()

            #lightring vs. robotdance
            # Advance to next state
            self.next_state = self.state2
            
        elif(self.curr_state == self.state2):
            # Wait for Nav2
            self.navigator.waituntilnav2active()
            
            # Set goal poses
            self.goal_pose = []
            self.add_goal(self.dest_positions['x1'], 
                          self.dest_positions['y1'],
                          self.dest_positions['direction1'])
            
            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Robot alerts user that they reached destination
            self.robot_talker(robot_phrase='We have reached your destination. Have a good day.')
            # Generate robot expression
            if(self.robot_expression == 'gesture'):
                self.generate_gesture_expression()
            elif(self.robot_expression == 'lightring'):
                self.generate_lightring_expression()
            else: 
                self.generate_gesture_lightring_expression()
                
            # Advance to next state
            self.next_state = self.state3
            
        elif(self.curr_state == self.state3):
            # Wait for Nav2
            self.navigator.waituntilnav2active()
            
            # Robot alerts user that it is departing
            self.robot_talker(robot_phrase='I am heading to my docking station.')
            
            # Set goal poses
            self.goal_pose = []
            self.add_goal(self.dest_positions['x2'], 
                          self.dest_positions['y2'],
                          self.dest_positions['direction2'])

            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Finished navigating to docking station
            self.navigator.dock()
            
            # Advance to next state
            self.next_state = self.state4
            
        elif(self.curr_state == self.state4):
            # Robot alerts user that it is done
            self.robot_talker(robot_phrase='My work is complete. Good day.')
            # End state machine
            self.next_state = None
            
        # Advance to next state
        self.curr_state = self.next_state

    def generate_gesture_expression(self):
        '''
        Generate gesture expression
        '''
        # Rotate clockwise
        self.move_robot(clockwise=True)
        # Rotate counterclockwise
        self.move_robot(clockwise=False)

    def generate_lightring_expression(self):
        # Create a ROS2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Override system lights
        lightring_msg.override_system = True

        # Sequence count
        self.sequence_count = 2

        for seq_num in range(self.sequence_count):
            # Alternate light on and off
            value = 255 if (seq_num % 2) == 1 else 0
            # LED 0
            lightring_msg.leds[0].red = value
            lightring_msg.leds[0].blue = 0
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = value
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 0
            lightring_msg.leds[2].green = value

            # LED 3
            lightring_msg.leds[3].red = value
            lightring_msg.leds[3].blue = value
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = value
            lightring_msg.leds[4].blue = 0
            lightring_msg.leds[4].green = value

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = value
            lightring_msg.leds[5].green = value

            # Publish the message
            self.lightring_publisher.publish(lightring_msg)
            time.sleep(0.1)
            lightring_msg.override_system = False
    
    def generate_gesture_lightring_expression(self):
        '''
        Generate both gesture and lightring expressions and 
        informs the user that they have reach their destination.
        '''
        # Generate lightring expression
        self.generate_lightring_expression()
        # Generate gesture expression
        self.generate_gesture_expression()

    def robot_talker(self, robot_phrase='Welcome to human robot interaction', output_filename='robot_talker.mp3'):
        """Uses text to speech software to enable to robot to 
            alert users when they are in the intimate and public zones                                                    
        ----------
        robot_phrase : robot phrase
            String of text phrase 
        output_filename : name of file to store audio file
            String of outputfile name
        Returns
        -------
        None
        """
        # Language in which you want to convert
        language = 'en'
        
        # Passing the text and language to the engine, 
        # here we have marked slow=False. Which tells 
        # the module that the converted audio should 
        # have a high speed
        myobj = gTTS(text=robot_phrase, lang=language, slow=False)
        
        # Saving the converted audio in a mp3 file named
        # welcome 
        myobj.save(output_filename)

        # Play audio file with playsound library
        playsound.playsound(output_filename, True)
    
    def move_robot(self, x=0.0, z=0.05, clockwise=True):
        """Move the robot using x and z velocities
        ----------
        x : float
            linear x velocity.
        z : float
            angualr z velocity.
        clockwise : bool
            True - rotate right, False - rotate left
        Returns
        -------
        None
        """
        self.move_cmd.linear.x = float(x) # back or forward
        if(clockwise):
            self.move_cmd.angular.z = float(-z)
        else:
            self.move_cmd.angular.z = float(z)
        self.movement_pub.publish(self.move_cmd)
        
    def add_goal(self, x, y, direction):
        '''`
        x : robot pose in x direction
        y : robot pose in y direction
        direction : orientation about the z axis in degress
        Returns None
        '''
        self.goal_pose.append(self.navigator.getPoseStamped([x,y], direction))

def main():
    rclpy.init()

    # Set goal poses
    user_position = {'x':-6.14, 'y':-4.65, 'direction':TurtleBot4Directions.NORTH}
    dest_position = {'x1':-5.25, 'y1':-5.75, 'direction1':TurtleBot4Directions.NORTH,
                     'x2':-4.2, 'y2':-6.2, 'direction2':TurtleBot4Directions.NORTH}

    # Set expressive behavior for robot. Options include gesture, lightring, or both
    robot_expression = 'gesture' # 'gesture', 'lightring', 'both'

    # Intiialize tour robot
    tour_robot = TourRobot(user_position, dest_position, robot_expression)

    # Run state_machine
    while(1):
        tour_robot.update_state_machine()
        if(tour_robot.curr_state is None):
            break

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
