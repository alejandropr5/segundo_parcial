import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import matplotlib.pyplot as plt
import numpy as np
 
class ImageSubscriber(Node): #Define an ImageSubscriber class, where the sub and process methods are going to be
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self): #Define the constructor of the class
    """
    Class constructor to set up the node
    """
    super().__init__('image_subscriber') #initialize the constructor whit image_subscriber as a Node
      
    self.subscription = self.create_subscription(
      Image, 
      'image_raw', 
      self.listener_callback,
      10) #Create a create_subscription object with Image as a type of msg, image_raw as the topic to subscribe, and a queue size of 10
    self.subscription # prevent unused variable warning

    #Create a create_publisher object with Image as a type of msg, create image_publisher as the topic to pub, and a queue size of 10
    self.image_pub = self.create_publisher(Image,'image_segmented',10) 
      

    self.br = CvBridge()  # Create a CvBridge object for the img process and convertions
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    
    current_frame = self.br.imgmsg_to_cv2(data)  # Convert ROS Image message to OpenCV image

    img_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB) #Convert the image from bgr to rgb
    hsv_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV) #Convert the image from rgb to hsv

    #Set minimum and maximum range 
    light_color = (50, 50, 50)
    dark_color = (150, 150, 255)

    mask = cv2.inRange(hsv_img, light_color, dark_color)    #create a binary mask which makes 1 every pixel inside the color range
    #mask2=((mask==0)*255).astype(np.uint8) 
    result = cv2.bitwise_and(img_rgb, img_rgb, mask=mask) #impose the mask on top of the original imag

    result_msg = self.br.cv2_to_imgmsg(result)
    result_msg.header.stamp = self.get_clock().now().to_msg()
    result_msg.header.frame_id = "default_cam"

    self.image_pub.publish(result_msg) #Publish the binary edge image in the topic previusly created
  
def main(args=None):  
  rclpy.init(args=args) 
  
  image_subscriber = ImageSubscriber() 
  
  
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()


