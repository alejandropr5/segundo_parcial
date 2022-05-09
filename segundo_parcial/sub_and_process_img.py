import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
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
    self.image_pub = self.create_publisher(Image,'image_processed',10) 
      

    self.br = CvBridge() # Create a CvBridge object for the img process and convertions
   
  def listener_callback(self, data):
    """
    Callback function.
    """

    current_frame = self.br.imgmsg_to_cv2(data) # Convert ROS Image message to OpenCV image
    
    
    edges = cv2.Canny(current_frame,100,200) #The binary edge image algorithm is made

    edges_rgb=cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR) #Convert 8UC1 to rgb
    edges_msg = self.br.cv2_to_imgmsg(edges_rgb,'rgb8') #Convert cv2 to imgmsg for publishing
    edges_msg.header.stamp = self.get_clock().now().to_msg() #Assing a stamp time
    edges_msg.header.frame_id = "default_cam" #define frame as a default_cam

    self.image_pub.publish(edges_msg) #Publish the binary edge image in the topic previusly created
  
def main(args=None):  
  rclpy.init(args=args) # Initialize the rclpy library
  
  image_subscriber = ImageSubscriber() # Create the node
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()