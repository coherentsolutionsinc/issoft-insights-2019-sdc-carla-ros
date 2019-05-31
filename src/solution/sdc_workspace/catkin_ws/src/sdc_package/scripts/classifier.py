import cv2
import numpy as np
import tensorflow as tf

traffic_pattern = -1

class Classifier(object):
    def __init__(self):
        print(tf.__version__)

        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default() as graph:
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile("../data/model.pb", 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

          with tf.Session() as sess:
            self.tensor_dict = {}
            self.tensor_dict['detection_scores'] = tf.get_default_graph().get_tensor_by_name('detection_scores:0')
            self.tensor_dict['detection_classes'] = tf.get_default_graph().get_tensor_by_name('detection_classes:0')    
            self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

        self.sess = tf.Session(graph=graph)                          

            
    def get_classification(self, image):
        global traffic_pattern

        image = cv2.resize(image[:,:,::-1],(400, 400), cv2.INTER_CUBIC)
        final_image = np.expand_dims(image, 0)

        output_dict = self.sess.run(self.tensor_dict, feed_dict={self.image_tensor: final_image})

        output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
        output_dict['detection_scores'] = output_dict['detection_scores'][0]

        score = output_dict['detection_scores'][0]

        print (output_dict)

        if score > 0.4:
            num = output_dict['detection_classes'][0] -1
        else:
            num = 6

        if score > 0.55 and traffic_pattern == -1:
            if num < 3:
                traffic_pattern = 1
            else:
                traffic_pattern = 2

            
        print("                                                                          ", num, score)

        if num == 6:
            return 4
        else:
            if (traffic_pattern == 1 and num < 3) or (traffic_pattern == 2 and num > 2) :
                return num % 3
            else:
                return 4
        return num 

if __name__ == '__main__':
    cl=Classifier()
    img = cv2.imread('../data/4.jpg', cv2.IMREAD_COLOR)
    cl.get_classification(img)
    cv2.imshow('image',img)

   