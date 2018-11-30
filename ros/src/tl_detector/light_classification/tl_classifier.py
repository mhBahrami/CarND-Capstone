import numpy as np
import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf

class TLClassifier(object):
    def __init__(self, is_site):
        """ 
        The constructor for TLClassifier class. 

        Parameters: 
           is_site (bool): Declares if simulated or site. 
        """
    
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        if is_site:
            PATH_TO_CKPT = r'models/real/frozen_inference_graph.pb'
        else:
            PATH_TO_CKPT = r'models/sim/frozen_inference_graph.pb'
            # PATH_TO_CKPT = r'models/sim/frozen_inference_graph_frcnn.pb'
        
        self.detection_graph = tf.Graph()
        
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')    
 
                # Definite input and output Tensors for detection_graph
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label. 	
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
         
        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.detection_graph.as_default():
            image_np_expand = np.expand_dims(image, axis=0)
        
            # Actual detection.
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expand})
                
        # Grab the top result
        class_state = np.squeeze(classes).astype(np.int32)[0]
        score = np.squeeze(scores)[0]
        
        
        # if class_state == 1:
        #     rospy.logwarn(">> {0} GREEN {1}".format(1, score))
        #     return TrafficLight.GREEN
        # elif class_state == 2:
        #     rospy.logwarn(">> {0} RED {1}".format(2, score))
        #     return TrafficLight.RED
        # elif class_state == 3:
        #     rospy.logwarn(">> {0} YELLOW {1}".format(3, score))
        #     return TrafficLight.YELLOW
        # else:
        #     rospy.logwarn(">> {0} UNKNOWN {1}".format(class_state, score))
        #     return TrafficLight.UNKNOWN
        if score > 0.2:
            if class_state == 1:
                # rospy.logwarn(">> {0} GREEN {1}".format(1, score))
                return TrafficLight.GREEN
            elif class_state == 2:
                # rospy.logwarn(">> {0} RED {1}".format(2, score))
                return TrafficLight.RED
            elif class_state == 3:
                # rospy.logwarn(">> {0} YELLOW {1}".format(3, score))
                return TrafficLight.YELLOW
        else:
            # rospy.logwarn(">> {0} UNKNOWN {1}".format(class_state, score))
            return TrafficLight.UNKNOWN
