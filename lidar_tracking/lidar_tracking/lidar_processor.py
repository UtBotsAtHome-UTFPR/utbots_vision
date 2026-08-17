import math

import cv2
import numpy as np

from lidar_tracking.sort import Sort


class LidarProcessor:

    def __init__(self):
        # Image settings
        self.img_size=240
        self.center = self.img_size // 2
        self.resolution=0.05

        # Morphology and detection settings
        self.kernel_size=3

        self.min_width=2
        self.max_width=8
        self.min_height=2
        self.max_height=8

        # Bounding box padding
        self.padding=10

        # SORT settings
        self.tracker = Sort(
            max_age=5,
            min_hits=1,
            iou_threshold=0.2
        )

    def process_scan(self, msg):
        """
        Processes a LaserScan and returns the current SORT trackers.

        Returns:
            current_tracks: dict
            img_color: OpenCV image for visualization
        """

        # RASTERIZATION (Lidar to Pixels)
        img = np.zeros((self.img_size, self.img_size),dtype=np.uint8)

        for i, range_val in enumerate(msg.ranges):

            if (np.isinf(range_val) or math.isnan(range_val) or range_val > msg.range_max or range_val < msg.range_min):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            pixel_x = self.center - int(y / self.resolution)

            pixel_y = self.center - int(x / self.resolution)

            if (0 <= pixel_x < self.img_size and 0 <= pixel_y < self.img_size):
                img[pixel_y, pixel_x] = 255

        # PRE-PROCESSING (Morphological Closing)
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        img = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)

        img_color = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

        # DETECTION AND FILTERING
        contours, _ = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for contour in contours:

            x, y, w, h = cv2.boundingRect(contour)

            # Filter: Only accepts if it is WITHIN the size window
            if (self.min_width <= w <= self.max_width and self.min_height <= h <= self.max_height):

                cv2.rectangle(img_color,(x, y),(x + w, y + h),(0, 255, 0),1)

                # Applies PADDING for SORT
                x1_pad = x - self.padding
                y1_pad = y - self.padding
                x2_pad = x + w + self.padding
                y2_pad = y + h + self.padding

                detections.append([x1_pad,y1_pad,x2_pad,y2_pad])
            else:
                cv2.rectangle(img_color,(x, y),(x + w, y + h),(0, 0, 255),1)

        # TRACKING (SORT)
        if len(detections) > 0:
            np_detections = np.asarray(detections,dtype=float).reshape(-1, 4)

        else:
            np_detections = np.empty((0, 4))

        tracked_objects = self.tracker.update(np_detections)

        # Convert SORT output into a convenient representation
        current_tracks = {}

        for obj in tracked_objects:

            x1, y1, x2, y2, obj_id = obj

            x1 = float(x1)
            y1 = float(y1)
            x2 = float(x2)
            y2 = float(y2)

            obj_id = int(obj_id)

            # Bounding box center
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            # DE-RASTERIZATION
            pos_x = (self.center - cy) * self.resolution

            pos_y = (self.center - cx) * self.resolution

            # Angular position relative to LiDAR
            cluster_angle = math.atan2(pos_y,pos_x)

            current_tracks[obj_id] = {
                "x": pos_x,
                "y": pos_y,
                "angle": cluster_angle,
                "bbox": (x1, y1, x2, y2)
            }

        return current_tracks, img_color