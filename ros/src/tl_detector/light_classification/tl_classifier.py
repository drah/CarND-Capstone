from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import tensorflow as tf
import cv2
from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self, pb_path):
        fetch_names = ['CBNOnet/light_state:0', 'CBNOnet/light_position:0']
        input_name = 'CBNOnet/images:0'
        self.sess = tf.Session()
        with tf.gfile.FastGFile(pb_path, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        self.sess.graph.as_default()
        tf.import_graph_def(graph_def, name='')
        self.fetch_nodes = [self.sess.graph.get_tensor_by_name(name) for name in fetch_names]
        self.fetch_dict = {node.name.split(':')[0].split('/')[-1]: node for node in self.fetch_nodes}
        self.input_node = self.sess.graph.get_tensor_by_name(input_name)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.resize(image,  (384, 288))
        image = np.expand_dims(image, 0)
        light_state, light_position = self.sess.run(
                self.fetch_dict,
                {self.input_node: image})
        return light_state
