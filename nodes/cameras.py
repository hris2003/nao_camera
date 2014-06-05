#!/usr/bin/env python

#
# ROS node to provide access to the depth and rgb topCamera 
#

from collections import defaultdict
import rospy
from sensor_msgs.msg import Image, CameraInfo
from nao_driver.nao_driver_naoqi import NaoNode
import camera_info_manager
from sensor_msgs.msg._CameraInfo import CameraInfo

from dynamic_reconfigure.server import Server
from nao_camera.cfg import NaoCameraConfig

from naoqi import ALProxy

from cv_bridge import CvBridge, CvBridgeError
import cv2
#import cv
import numpy as np

# import resolutions
from nao_camera.vision_definitions import k960p, k4VGA, kVGA, kQVGA, kQQVGA
# import color spaces
from nao_camera.vision_definitions import kYUV422ColorSpace, kYUVColorSpace, \
                    kRGBColorSpace, kBGRColorSpace, kDepthColorSpace
# import extra parameters
from nao_camera.vision_definitions import kCameraSelectID, kCameraAutoExpositionID, kCameraAecAlgorithmID, \
                  kCameraContrastID, kCameraSaturationID, kCameraHueID, kCameraSharpnessID, kCameraAutoWhiteBalanceID, \
                  kCameraExposureID, kCameraGainID, kCameraBrightnessID, kCameraWhiteBalanceID

# those should appear in vision_definitions.py at some point
kTopCamera = 0
kBottomCamera = 1
kDepthCamera = 2

class NaoCam (NaoNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_camera')

        self.camProxy = self.getProxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)
        self.nameId = None
        self.camera_infos = {}
        def returnNone():
            return None
        self.config = defaultdict(returnNone)

        self.bridge = CvBridge()
        # ROS publishers
        self.pub_img_ = rospy.Publisher('~image_raw', Image)#for depth image
        self.pub_cimg_ = rospy.Publisher('~image_color', Image)#for color image
        self.pub_depth_info_ = rospy.Publisher('~camera_info', CameraInfo)
        
        # initialize the parameter server
        self.srv = Server(NaoCameraConfig, self.reconfigure)


    def reconfigure(self, new_config, level):
        '''trying to configure the color camera and the depth camera together to get the two images at the same time'''
        if self.pub_img_.get_num_connections() == 0:
            rospy.loginfo('Changes recorded but not applied as nobody is subscribed to the ROS topics.')
            self.config.update(new_config)
            return self.config
	
        # check if we are even subscribed to a camera
        is_camera_new = self.nameId is None
        self.frame_id = "/CameraTop_frame"#new_config['camera3d_frame']
        
        if new_config['source'] > 1:#not topcam or bottomcam
	    rospy.logerr('Invalid source. Must be 0 or 1')
            return self.config
            
	curRgbCam = new_config['source']
	
        if self.config['source'] != new_config['source'] or is_camera_new:
            # unsubscribe for all zombie subscribers
            self.camProxy.unsubscribeAllInstances("rospy_gvm")
            # subscribe
            self.nameId = self.camProxy.subscribeCameras("rospy_gvm", [curRgbCam, kDepthCamera], [new_config['resolution'],new_config['resolution']], [new_config['color_space'],17],
                                                  new_config['frame_rate'])
            rospy.loginfo('Using camera: %d . Subscriber name is %s .' % (new_config['source'], self.nameId))

	
        # check if the camera_info_url changed
        # actually, we dont care about the camera_info_url
        '''
        if self.config['camera_info_url'] != new_config['camera_info_url'] and \
                        new_config['camera_info_url'] and new_config['camera_info_url'] not in self.camera_infos:
        
	    if 'cim' not in self.__dict__:
		self.cim = camera_info_manager.CameraInfoManager(cname='nao_camera')
        
            if not self.cim.setURL( new_config['camera_info_url'] ):
                rospy.logerr('malformed URL for calibration file')
            else:
                try:
                    self.cim.loadCameraInfo()
                except IOExcept:
                    rospy.logerr('Could not read from existing calibration file')

            if self.cim.isCalibrated():
                rospy.loginfo('Successfully loaded camera info')
                self.camera_infos[new_config['camera_info_url']] = self.cim.getCameraInfo()
            else:
                rospy.logerr('There was a problem loading the calibration file. Check the URL!')
        '''
        
        # set params
        for key, naoqi_key in [('source', kCameraSelectID), ('auto_exposition', kCameraAutoExpositionID),
                               ('auto_exposure_algo', kCameraAecAlgorithmID),
                               ('contrast', kCameraContrastID), ('saturation', kCameraSaturationID),
                               ('hue', kCameraHueID), ('sharpness', kCameraSharpnessID),
                               ('auto_white_balance', kCameraAutoWhiteBalanceID)
                               ]:
            if self.config[key] != new_config[key] or is_camera_new:
                self.camProxy.setParameter(curRgbCam, naoqi_key, new_config[key])
                self.camProxy.setParameter(kDepthCamera, naoqi_key, new_config[key])

        for key, naoqi_key, auto_exp_val in [('exposure', kCameraExposureID, 0),
                                             ('gain', kCameraGainID, 0), ('brightness', kCameraBrightnessID, 1)]:
            if self.config[key] != new_config[key] or is_camera_new:
                self.camProxy.setParameter(curRgbCam, kCameraAutoExpositionID, auto_exp_val)
                self.camProxy.setParameter(curRgbCam, naoqi_key, new_config[key])
                self.camProxy.setParameter(kDepthCamera, kCameraAutoExpositionID, auto_exp_val)
                self.camProxy.setParameter(kDepthCamera, naoqi_key, new_config[key])

        if self.config['white_balance'] != new_config['white_balance'] or is_camera_new:
            self.camProxy.setParameter(curRgbCam, kCameraAutoWhiteBalanceID, 0)
            self.camProxy.setParameter(curRgbCam, kCameraWhiteBalanceID, new_config['white_balance'])
            self.camProxy.setParameter(kDepthCamera, kCameraAutoWhiteBalanceID, 0)
            self.camProxy.setParameter(kDepthCamera, kCameraWhiteBalanceID, new_config['white_balance'])

        for key, method in [('resolution', 'setResolutions')]:
            if self.config[key] != new_config[key] or is_camera_new:
	        rs =self.camProxy.__getattribute__(method)(self.nameId, [new_config[key],new_config[key]])
		
        if self.config['color_space'] != new_config['color_space'] or is_camera_new:
	    self.camProxy.setColorSpaces(self.nameId, [new_config['color_space'],17])
	    
	if self.config['frame_rate'] != new_config['frame_rate'] or is_camera_new:
	    self.camProxy.setFrameRate(self.nameId, new_config["frame_rate"])
                           
        self.config.update(new_config)

        return self.config

    '''we are trying to get images from the color camera and the depth camera'''
    def main_loop(self):
        img = Image()
        cimg = Image()
        r = rospy.Rate(self.config['frame_rate'])
        while not rospy.is_shutdown():
            if self.pub_img_.get_num_connections() == 0:
                if self.nameId:
                    rospy.loginfo('Unsubscribing from camera as nobody listens to the topics.')
                    self.camProxy.unsubscribe(self.nameId)
                    self.nameId = None
                r.sleep()
                continue
            if self.nameId is None:
                self.reconfigure(self.config, 0)
                
                r.sleep()
                continue
            images = self.camProxy.getImagesRemote(self.nameId)
            
            if images is None:
                continue
	      
	    image = images[0]
	    if image[3] == kDepthColorSpace:
                depthId = 0
            else:
		depthId = 1
            #else:
                #rospy.logerr("Received unknown encoding: {0}".format(image[3]))
                
            image = images[depthId]
            ####  # Deal with the depth image
	    if self.config['use_ros_time']:
		img.header.stamp = rospy.Time.now()
	    else:
		img.header.stamp = rospy.Time(image[4] + image[5]*1e-6)
	    img.header.frame_id = self.frame_id
	    img.height = image[1]
	    img.width = image[0]
	    nbLayers = image[2]

	    if image[3] == kDepthColorSpace:
		encoding = "mono16"
	    else:
		rospy.logerr("Received unknown encoding: {0}".format(image[3]))
	    img.encoding = encoding
	    img.step = img.width * nbLayers
	    img.data = image[6]	      
	    self.pub_img_.publish(img)
	    
	    image = images[1-depthId]
	    ######  # Deal with the color image
	    if self.config['use_ros_time']:
		cimg.header.stamp = rospy.Time.now()
	    else:
		cimg.header.stamp = rospy.Time(image[4] + image[5]*1e-6)
	    cimg.header.frame_id = self.frame_id
	    cimg.height = image[1]
	    cimg.width = image[0]
	    nbLayers = image[2]
	    if image[3] == kYUVColorSpace:
		encoding = "mono8"
	    elif image[3] == kRGBColorSpace:
		encoding = "rgb8"
	    elif image[3] == kBGRColorSpace:
		encoding = "bgr8"
	    elif image[3] == kYUV422ColorSpace:
		encoding = "yuv422" # this works only in ROS groovy and later
	    elif image[3] == kDepthColorSpace:
		encoding = "mono16"
	    else:
		rospy.logerr("Received unknown encoding: {0}".format(image[3]))
	    cimg.encoding = encoding
	    cimg.step = cimg.width * nbLayers
	    cimg.data = image[6]
            self.pub_cimg_.publish(cimg)
            
            # deal with the depth camera info
	    infomsg = CameraInfo()
	    # yes, this is only for an XTion / Kinect but that's the only thing supported by NAO
	    ratio_x = float(640)/float(img.width)
	    ratio_y = float(480)/float(img.height)
	    infomsg.width = img.width
	    infomsg.height = img.height
	    # [ 525., 0., 3.1950000000000000e+02, 0., 525., 2.3950000000000000e+02, 0., 0., 1. ]
	    infomsg.K = [ 525, 0, 3.1950000000000000e+02,
			  0, 525, 2.3950000000000000e+02,
			  0, 0, 1 ]
	    infomsg.P = [ 525, 0, 3.1950000000000000e+02, 0,
			  0, 525, 2.3950000000000000e+02, 0,
			  0, 0, 1, 0 ]
	    for i in range(3):
		infomsg.K[i] = infomsg.K[i] / ratio_x
		infomsg.K[3+i] = infomsg.K[3+i] / ratio_y
		infomsg.P[i] = infomsg.P[i] / ratio_x
		infomsg.P[4+i] = infomsg.P[4+i] / ratio_y

	    infomsg.D = []
	    infomsg.binning_x = 0
	    infomsg.binning_y = 0
	    infomsg.distortion_model = ""
	    self.pub_depth_info_.publish(infomsg)

            r.sleep()

        self.camProxy.unsubscribe(self.nameId)

if __name__ == "__main__":
    try:
        naocam = NaoCam()
        naocam.main_loop()
    except RuntimeError as e:
        rospy.logerr('Something went wrong: %s' % str(e) )
    rospy.loginfo('Camera stopped')
