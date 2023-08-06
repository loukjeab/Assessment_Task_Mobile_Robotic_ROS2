#!/usr/bin/env python3

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from visualization_msgs.msg import Marker # Import the Marker message type from visualization_msgs package
from geometry_msgs.msg import Pose, Point, Quaternion # Import the Pose, Point, and Quaternion message types from geometry_msgs package
import cv2  # Import the OpenCV library
import cv2 # OpenCV library
import numpy as np # Import the NumPy library for numerical operations
import math # Import the math module for mathematical operations

# Print "Hello!" to terminal
print("Hello!")

PositionMarked = 1
 
 
class ArrowMarkerPublisher(Node):
    def __init__(self):
        """
        Class constructor to set up the node.
        """
        super().__init__('arrow_marker_publisher') # Initialize the parent class Node with the name 'arrow_marker_publisher'
        self.publisher_ = self.create_publisher(Marker, 'arrow_marker', 10) # Create a publisher for 'arrow_marker' topic with Marker
        self.marker_id = 0 # initializes the marker ID counter
        print("Marker init!")
 

    def publish_marker(self, xpos, ypos):
        print("Start publishing marker.")
        marker = Marker() # Create a Marker object
        marker.header.frame_id = 'base_link' # Set the frame ID for the marker
        marker.type = Marker.CYLINDER # Set the marker type to CYLINDER
        marker.id = self.marker_id # Set the ID for the marker

        # Set the x y z coordinate of the marker's position
        marker.pose.position.x = xpos 
        marker.pose.position.y = ypos
        marker.pose.position.z = 0.5
        marker.scale.x = 0.3  # Shaft diameter
        marker.scale.y = 0.3  # Head diameter
        marker.scale.z = 1.0  # Head length
        # Set the red green blue component of the marker's color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0 # Set the alpha component (transparency) of the marker's color

        self.publisher_.publish(marker) # Publish the marker
        self.marker_id += 1 # Increment the marker ID
        print("End publishing marker.")
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(Image, 'camera1/image_raw', self.listener_callback, 10)
    # prevent unused variable warning
    self.subscription 
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()


  def listener_callback(self, data):
    """
    Callback function.
    """

    # Display a log message indicating the reception of a video frame
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
    
    # Process the image to get yellow object coordinates
    get_yellow_object_coordinates(current_frame_rgb)

    # Display the image (optional)
    #cv2.imshow("camera", current_frame)
    #cv2.imshow("camera", current_frame_rgb)

    # Wait for a key press (to handle the OpenCV windows)
    cv2.waitKey(1)
    
      
def get_yellow_object_coordinates(image):
    global PositionMarked

    # Define the color range for yellow in the BGR color space
    lower_yellow = np.array([0, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    # Create a mask for the yellow color area
    yellow_mask = cv2.inRange(image, lower_yellow, upper_yellow)

    # Find contours in the image
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Sort the contours by their area in descending order
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    print("Contours =",len(contours))
    
    if len(contours)>0: 
        # Extract the contour with the largest area
        largest_contour = contours[0]

        # Calculate the bounding box around the contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        print("w =",w)
        print("PositionMarked =",PositionMarked)
        
        # If the width of the bounding box is greater than 100 pixels and there are remaining positions to mark
        if w>100 and PositionMarked>0:

            print("Yellow Block Found")
            PositionMarked = PositionMarked -1

            # Extract the number of X and Y pixels
            height, width, _ = image.shape
    
            # Check if the object is in the center of the picture
            if(x>=height/4) and (x+w<=(height/4)*3):
                inCenterOfPicture=1
            else:
                inCenterOfPicture=0
    
            # Calculate the distance and angle to the yellow object
            object_coordinates = x, y, x+w, y+h ,width ,height, inCenterOfPicture 
            yellow_position = calculate_distance_compesated(object_coordinates)
            fxpos = float(math.cos(yellow_position[1]) * yellow_position[0]) 
            fypos = float(math.sin(yellow_position[1]) * yellow_position[0])

            print("Calculation done x ",fxpos) 
            print("Calculation done y ",fypos)

            # Create an instance of ArrowMarkerPublisher and publish the marker
            marker_obj = ArrowMarkerPublisher()
            print("Marker Created")
            marker_obj.publish_marker(xpos=fxpos, ypos=fypos)
    

def calculate_distance_compesated(object_coordinates):
    hoizontal_fow = 1.3962634 # Horizontal field of view in radians
    vertical_fow = hoizontal_fow # Vertical field of view (assumed to be the same as horizontal)
    
    # Calculate the center coordinates of the object
    y=object_coordinates[1]+(object_coordinates[3]-object_coordinates[1])/2  # y-coordinate plus half of the height
    x=object_coordinates[0]+(object_coordinates[2]-object_coordinates[0])/2  # x-coordinate plus half of the width
    
    Pixel_Nr_X          = object_coordinates[4] # Number of pixels in the X dimension (width) 
    Pixel_Nr_Y          = object_coordinates[5] # Number of pixels in the Y dimension (height) 
    camera_Hight        = 0.25  # Camera height in meters
    Offset_Winkel_rad   = 0 # Offset angle in radians (optional) #-0.78 #kammera 45° negativ gegen horizontal  
    
    winkel_rad = 0
    
    # Calculate the distance to the virtual far plane in pixels
    distance_in_px_relative_to_virtual_far_plane = (Pixel_Nr_Y/2) / math.tan((hoizontal_fow/2)) 
    
    # Calculate the vertical distance from the object's center to the virtual far plane
    yunterehalfte=y-(Pixel_Nr_Y/2)
    
    #print("yunterehalfte:",yunterehalfte)
    
    # Calculate the distance to the virtual far plane in pixels for the x-dimension
    distance_in_px_relative_to_virtual_far_plane_for_x = (Pixel_Nr_X/2) / math.tan((vertical_fow/2)) 
    
    # Determine the half of the maximum pixels and subtract or invert based on the object's x-coordinate
    if(x>=(Pixel_Nr_X/2)):
        x_halfte=x-(Pixel_Nr_X/2)  #halfte der max px abziehen
    else:
        x_halfte=(Pixel_Nr_X/2)-x  #richtung invertieren
    
    #print("x_halfte:",x_halfte)
    
    # Calculate the angle for the x-dimension relative to the virtual far plane
    x_winkel_rad=math.atan2(x_halfte , distance_in_px_relative_to_virtual_far_plane_for_x)
    
    # Calculate the angle for the y-dimension relative to the virtual far plane
    winkel_rad=math.atan2(yunterehalfte , distance_in_px_relative_to_virtual_far_plane)
    winkel_rad= -winkel_rad # Invert the angle
    winkel_rad=winkel_rad+Offset_Winkel_rad+1.57 # Add offset angle and 90 degrees in radians #+90 grad 
    
    # Calculate the distance to the object using tangent function and camera height
    abstand = math.tan(winkel_rad) * camera_Hight

    print("Abstand ohne seitlicher comp: {:.2f} m".format(abstand) )

    # Apply side compensation by dividing the distance to the virtual far plane by the cosine of the x-angle
    distance_in_px_relative_to_virtual_far_plane=distance_in_px_relative_to_virtual_far_plane / math.cos(abs(x_winkel_rad))  #seitliche kompensation 
    
    # Calculate the angle for the y-dimension relative to the virtual far plane with side compensation
    winkel_fur_pixel=math.atan2(yunterehalfte , distance_in_px_relative_to_virtual_far_plane)
    
    winkel_rad= -winkel_fur_pixel # Invert the angle
    winkel_rad=winkel_rad+Offset_Winkel_rad+1.57 # Add offset angle and 90 degrees in radians #+90 grad
    
    # Calculate the distance to the object with side compensation using tangent function and camera height
    abstand = math.tan(winkel_rad) * camera_Hight

    #print("Abstand mit seitlicher comp:", abstand)
    print("Abstand mit seitlicher comp : {:.2f} m".format(abstand) )

    print("x_winkel_rad : {:.3f} rad:".format(x_winkel_rad))
    return (abstand,x_winkel_rad)
    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()

  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
