import sys
import geometry_msgs.msg
import rclpy
import termios
import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages.
---------------------------
Moving around:
        w     
   a    s    d
        x     
---------------------------
"""


move_bindings = {
    "a": (0, 0, 0, 1),
    "w": (1, 0, 0, 0),
    "s": (0, 0, 0, 0),
    "d": (0, 0, 0, -1),
    "x": (-1, 0, 0, 0),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(x, z):
    return "currently:\tspeed %s\tturn %s " % (round(x, 2), round(z, 2))


def limit(value, l):
    if value > l:
        return l

    elif value < -l:
        return -l

    else:
        return value


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("teleop_keyboard_node")
    pub = node.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", 10)

    x = 0.0
    th = 0.0
    x_factor = 0.1
    th_factor = 0.1

    status = 0

    try:
        print(msg)
        print(vels(x, th))

        while True:
            key = getKey(settings)
            if key in move_bindings.keys():

                # ==========================================================
                # TODO 1
                #
                # Process keyboard input and update the rover's motion.
                #
                # Requirements:
                # - Determine the requested motion using move_bindings.
                # - Update the forward and steering velocities.
                # - Clamp both values within [-1.0, 1.0].
                # - Reset the velocities when the stop key is pressed.
                #
                # Hint:
                # Use:
                #   • move_bindings
                #   • limit()
                #   • x_factor
                #   • th_factor
                # ==========================================================
                
                # YOUR CODE HERE
                if key == "s":
                    x = 0.0
                    th = 0.0
                else:
                    x = limit(x + move_bindings[key][0] * x_factor, 1.0)
                    th = limit(th + move_bindings[key][3] * th_factor, 1.0)
                    
                if status == 14:
                    print(msg)
                status = (status + 1) % 15

                print(vels(x, th))

            elif key == "\x03":
                break

            # ==========================================================
            # TODO 2
            #
            # Create and publish a geometry_msgs.msg.Twist message.
            #
            # Requirements:
            # - Create a Twist message.
            # - Set the rover's forward velocity using the current
            #   linear velocity.
            # - Set the rover's steering command using the current
            #   angular velocity.
            # - Keep all remaining Twist fields equal to zero.
            # - Publish the completed message to the /cmd_vel topic.
            #
            # Hint:
            # Only linear.x and angular.z should change. Use the
            # publisher created earlier in the node to send the
            # message.
            # ==========================================================

            # YOUR CODE HERE
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th
            pub.publish(twist)
            
    except Exception as e:
        print(e)

    finally:
        
        # ==========================================================
        # TODO 3
        #
        # Safely stop the rover before exiting.
        #
        # Requirements:
        # - Create a Twist message with all velocity components
        #   set to zero.
        # - Publish the message once before shutting down.
        #
        # Why?
        # Publishing a zero velocity command ensures the rover
        # stops immediately when the teleoperation node exits.
        #
        # Hint:
        # Create a geometry_msgs.msg.Twist() object and set all
        # linear and angular velocity components to zero before
        # publishing.
        # ==========================================================

        # YOUR CODE HERE
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0        
        pub.publish(twist)

        restoreTerminalSettings(settings)

if __name__ == "__main__":
    main()
