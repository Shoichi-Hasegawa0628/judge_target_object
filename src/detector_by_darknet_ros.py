import datetime
import sys

import actionlib
from timeit import default_timer as timer
from sensor_msgs.msg import RegionOfInterest
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal, CheckForObjectsResult
from hsr_common.msg import BoundingBox2D


class DetectorByDarknetROS(object):

    def __init__(self, server_name):
        """
        :type server_name: str
        """
        self._client = actionlib.SimpleActionClient(server_name, CheckForObjectsAction)
        print('Waiting for the server "{}"...'.format(server_name))
        self._client.wait_for_server()
        print('Connected to the service "{}"...'.format(server_name))

    def detect(self, img):
        """
        :type img: Image
        """
        goal = CheckForObjectsGoal()
        goal.image = img
        goal.id = int(datetime.datetime.now().strftime("%S%Z%z%f")) % 65535 - 32768

        start = timer()
        # Send request.
        while True:
            self._client.send_goal(goal)
            self._client.wait_for_result()
            raw_res = self._client.get_result()
            print(goal.id, raw_res.id)
            if raw_res.id == goal.id:
                break

        # Convert 'raw_data'.
        conversion_res = self._translate_response(raw_res)

        end = timer()
        print('prediction finished. ({} [sec])'.format(end - start))
        sys.stdout.flush()

        return conversion_res

    @staticmethod
    def _translate_response(raw_res):
        """
        :type raw_res: CheckForObjectsResult
        """
        bboxes = raw_res.bounding_boxes.bounding_boxes
        conversion_res = [BoundingBox2D() for i in range(len(bboxes))]
        for i, bbox in enumerate(bboxes):
            bounding_box = conversion_res[i]
            roi_param = {
                'x_offset': bbox.xmin,
                'y_offset': bbox.ymin,
                'height': bbox.ymax - bbox.ymin + 1,
                'width': bbox.xmax - bbox.xmin + 1,
                'do_rectify': True
            }
            bounding_box.region = RegionOfInterest(**roi_param)
            bounding_box.name = bbox.Class
            bounding_box.score = bbox.probability
            bounding_box.label = 0

        return conversion_res
