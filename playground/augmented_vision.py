import cv2 as cv
import numpy as np
import pyreal as rs
import math

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

# FPS set to 30
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)

color_info = (0, 0, 255)
rayon = 10

# Template matching setup
img_rgb_template = cv.imread('car.png')
img_gray_template = cv.cvtColor(img_rgb_template, cv.COLOR_BGR2GRAY)
template = cv.imread('template.png', 0)
w, h = template.shape[::-1]
threshold_template = 0.3

# Segmentation parameters using HSV color space
color = 60 # Adjust as needed
lo = np.array([color - 15, 50, 50]) # Adjust as needed
hi = np.array([color + 6, 255, 255]) # Adjust as needed

# Flag to check if the template is detected
object_detected = False

# Distance threshold for object detection (increased distance)
distance_threshold = 1.0  # Adjust as needed

# Creating morphological kernel
kernel = np.ones((3, 3), np.uint8)

try:
    while True:
        # RealSense frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame:
            continue

        # Colorized depth map
        colorized_depth = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(colorized_depth.get_data())

        # Set background to black
        depth_colormap[depth_colormap == 0] = 0

        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(aligned_color_frame.get_data())

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # Use pixel value of depth-aligned color image to get 3D axes
        x, y = int(color_colormap_dim[1] / 2), int(color_colormap_dim[0] / 2)
        depth = depth_frame.get_distance(x, y)
        dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)
        distance = math.sqrt(((dx) ** 2) + ((dy) ** 2) + ((dz) ** 2))

        # Check if the object is within the specified distance
        if distance < distance_threshold:
            # Convert color image to HSV
            hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

            # Segmentation in HSV color space
            mask = cv.inRange(hsv_image, lo, hi)
            mask = cv.erode(mask, kernel, iterations=1)
            mask = cv.dilate(mask, kernel, iterations=1)
            image_segmented = cv.bitwise_and(color_image, color_image, mask=mask)

            # Template matching with segmented image
            img_gray_segmented = cv.cvtColor(image_segmented, cv.COLOR_BGR2GRAY)
            res = cv.matchTemplate(img_gray_segmented, template, cv.TM_CCOEFF_NORMED)
            loc = np.where(res >= threshold_template)

            # Reset the object_detected flag
            object_detected = False

            for pt in zip(*loc[::-1]):
                cv.rectangle(color_image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
                object_detected = True

            # Show images
            images = np.hstack((color_image, depth_colormap, image_segmented))

            cv.circle(images, (int(x), int(y)), int(rayon), color_info, 2)
            cv.circle(images, (int(x + color_colormap_dim[1]), int(y)), int(rayon), color_info, 2)

            cv.putText(images, "D=" + str(round(distance, 2)), (int(x) + 10, int(y) - 10),
                        cv.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv.LINE_AA)
            cv.putText(images, "D=" + str(round(distance, 2)), (int(x + color_colormap_dim[1]) + 10, int(y) - 10),
                        cv.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv.LINE_AA)

            # Show images
            cv.imshow('RealSense', images)
            cv.waitKey(1)

            # Print object detection status in real-time
            if object_detected:
                print("Object detected!")
            else:
                print("Object not detected.")

except Exception as e:
    print(e)

finally:
    pipeline.stop()