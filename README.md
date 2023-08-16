# image_processing
This ROS node processes video feed from multiple cameras to detect the presence of specific colors: Red, Green, and Blue. Leveraging the power of OpenCV and the flexibility of the HSV color space.

## Setup and configuration

    git clone https://github.com/Team-Ausdroid-Unimelb/image_processing.git

Navigate to ``../image_processing/config/`` to configure the appropriate params for you. In the ``camera_config.yaml`` file, you can decalre which one or more cameras are used to detect. In Linux system, you can check available camera devices in command line using 

    ls /dev/video*

Sometimes, you may see multiple video devices for a single camera, check [here](https://askubuntu.com/questions/1123601/four-dev-video-entries-but-just-one-camera) for more details.

In the ``HSV_filter_param.yaml`` file, you can tune the thereshold for your application. Each color has a lower and an upper threshold, defining a range in the HSV space that represents that particular color.

Format:

`<Color>_lower_threshold: [Hue_min, Saturation_min, Value_min]`
`<Color>_upper_threshold: [Hue_max, Saturation_max, Value_max]`

**Hue (H)**: Represents the type of color (e.g., red, green, or blue). It's measured in degrees on the color circle ranging from 0° to 360°. However, in OpenCV, it's scaled to 0 to 180 to fit into 8-bit value.

**Saturation (S)**: Represents the richness of the color, ranging from 0 (grey) to 255 (vivid).

**Value (V)**: Represents the brightness of the color, ranging from 0 (black) to 255 (bright).

For more details on HSV, please check the [Wiki](https://en.wikipedia.org/wiki/HSL_and_HSV) page and [OpenCV](https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html) page.

## Launch node
A sample launch file is provided in ``../image_processing/launch/``, the ``verbose_original`` and ``verbose_filter`` toggles whether to show the video output. They are deafult to ``False`` to save processing time. Set them to ``True`` if you would like to see the results.

## Publisher
Three publishers are included in this ros pack.

* detection_robot_r
* detection_robot_g
* detection_robot_b

The current algorithm evaluate all cameras as one big frame. If any of them detect the specific color, the results will be set to true. The publisher will only publish the messages once the state changes. 