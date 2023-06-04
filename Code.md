# ROS Python Code
The turtlebot follower ROS package makes use of the [Open Computer Vision](https://opencv.org/) (OpenCV) software and python language to collect images from a camera, extract features from the images and move the turtlebot accordingly. The ultimate strategy in setting the following algorithm for the turtlebot is to detect a pattern of colour that is mounted on the back of the guide robot. This pattern is detected by processing the image information collected from the camera, by means of sampling the colours extracted from the environment and masking them to a binary format (black & white). From the binary image, a centroid can be calculated distant from the camera centre, which communicates the velocities of the follower turtlebot.
Thus, the package contains 4 nodes in total, 2 for colour detection and the remaining 2 for feature extraction and movement. 

## Image Collection and Conversion
The camera used is the [Intel RealSense R200](https://www.intel.com.au/content/www/au/en/support/products/92256/emerging-technologies/intel-realsense-technology/intel-realsense-cameras/intel-realsense-camera-r200.html), with 1 RGB (R-Red, G-Green, B-Blue) Camera, and 2 Infra-Red cameras. The RGB images collected from the camera are then encoded into the BGR format which is the default format supported by OpenCV. The main function of the node is to subscribe to the corresponding camera topic that extracts the RBG images. For the R200, the topic subscribed to is `'/camera/color/image_raw'`. This image once collected is then converted to the BGR format as shown in the following line (From Line 30 in the `camera_collection.py` node):

                        self.cv_image = self.bridge_object.imgmsg_to_cv2(raw_cv_image, desired_encoding="bgr8")

## Feature Detection and Image Processing
Once the images are collected and converted to BGR they can be loaded into OpenCV. OpenCV allows for easy conversion of images and through it, the BGR images are then  converted to the HSV (H-Hue, S-Saturation, V-Value) format. The benefit of using the HSV as compared to RGB is due to its ability to separate image luma (luminance) from chroma (colour information), making it easier to work with setups in need of luminance in the images. For this application, it is used for colour thresholding based on luminosity. The node `hsvcolor_detector.py` picks up the converted HSV image and samples the colours in it based on the thresholds provided. An illustration of its UI is shown below: <br/>

   ![Screenshot from 2022-10-25 02-47-04](https://user-images.githubusercontent.com/113515994/197569389-a7ca7109-29a7-4700-abc8-31a5cfb85985.png)

If the right thresholds are used for the features those will show up in their respective colour and the rest will show up as black, massively reducing the time taken to process the important details of the image. For our application, the feature to be detected in the environment is a blue circle. For the colour blue, the lower and upper thresholds are:
- Lower threshold = [104,134,0]
- Upper threshold = [111,255,255]

To further reduce the time it takes to process the image, we mask the sampled image to a binary format, where white represents the desired pattern and black the unnecessary information. With OpenCV, Bit Masking is applied which stores the image data as bits operating on boolean logic. This is shown in Line 61 of the `feature_follower.py` node:

                                          cv2.bitwise_and(cv_image,cv_image, mask= mask)

This processed image can then be used to generate outputs which will serve as velocity commands inputted to the Turtlebot.

## Turtlebot Movement
In moving the turtlebot, it is necessary to first calculate the distance between the features in the masked image and the camera mounted on the turtlebot. This can be achieved by calculating the centroid of the features to allow the turtlebot to be centred with the feature and measuring the distance between the centroid blob to the camera lens (**Note:** The R200 is a stereo camera, thus it is capable of obtaining depth information), ensuring that the feature is within its 2.0 m range. This distance information is then compiled to trace the necessary linear and angular velocities to keep the feature within the 2 m depth range. <br/>
With the velocities sent to the robot, it's important to consider how the motors of the wheels are then controlled. For our application, a proportional (P) controller was used to control the turtlebot as it was easy to design and even with its error margins, it was still accurate enough to move the robot in the desired path, further improved by the image processing. Ideally, a [PID controller](https://www.youtube.com/watch?v=okqIgZJy67E) would be desirable, as it provides the motor torque required for the robot to achieve its desired position and velocity.

# MATLAB Code
The MATLAB implementation is fairly similar to the ROS Package previously described in terms of image processing, with the only difference being  that the image is not masked. To carry out the image processing, instead of converting the images to HSV they are converted to L* a* b* (L*-Lightness, a*-Red/Green value, b*-Blue/Yellow value). L* a* b* colour is a more accurate colour space compared to RGB, and it’s device-independent, meaning that it’s easier to achieve exactly the same colour across different media. Minimum and maximum thresholds for the colour desired are also set, and for the blue colour these are:
- Lower threshold = [0,-100,-100]
- Upper threshold = [50,100,-15]

With this sampling, the details in the image that are not part of the threshold are still visible meaning that more information must be processed to detect the right colour feature in the environment. This also inhibits the possibility of masking meaning that the centroid calculation will be done with the image filled with all the original details it encompasses. <br/>
Similar to the ROS Package, the distances in the camera depth range can then be calculated and velocity commands are sent to the turtlebot, which uses a proportional controller to control the motors.
 
# Package Details
Dependencies: `rospy`, `cv_bridge`, `image_transport`, `sensor_msgs`. <br/>
Intel RealSense R200 Package: [librealsense](http://wiki.ros.org/librealsense). Compatible with ROS Kinetic (installed on Intel Board)
