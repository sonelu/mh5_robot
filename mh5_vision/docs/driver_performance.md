
# Camera Performance

## Get Camera Information

List cameras with `v4l2-ctl --list-devices`:

    pi@MH5:~/project_ws $ v4l2-ctl --list-devices
    bcm2835-codec-decode (platform:bcm2835-codec):
        /dev/video10
        /dev/video11
        /dev/video12

    HBV HD CAMERA: HBV HD CAMERA (usb-0000:01:00.0-1.1):
        /dev/video0
        /dev/video1

    HBV HD CAMERA: HBV HD CAMERA (usb-0000:01:00.0-1.2):
        /dev/video2
        /dev/video3

List device capabilities with `v4l2-ctl -d /dev/videoX --list-formats-ext`

    pi@MH5:~/project_ws $ v4l2-ctl -d /dev/video0 --list-formats-ext
    ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
            Size: Discrete 1920x1080
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 1280x720
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 640x480
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 352x288
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 320x240
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 176x144
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 160x120
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 1920x1080
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
            Size: Discrete 640x480
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 352x288
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 320x240
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 176x144
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 160x120
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
            Size: Discrete 640x480
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)
                Interval: Discrete 0.033s (30.000 fps)
                Interval: Discrete 0.040s (25.000 fps)
                Interval: Discrete 0.050s (20.000 fps)
                Interval: Discrete 0.067s (15.000 fps)
                Interval: Discrete 0.100s (10.000 fps)
                Interval: Discrete 0.200s (5.000 fps)

And:

    pi@MH5:~/project_ws $ v4l2-ctl -d /dev/video1 --list-formats-ext
    ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

## Performance Evaluation

We'll focus on the MJPG capture as this offers better bandwidth.

Run `htop` and record the 1m load after a period of 10 minutes of idle state: 0.03.

ROS node is run with the given parameters and on a remote machine `rqt` is started where `Topic Monitor` and `Image View` plug-ins are run. We track in `Topic Monitor` `/sensor_msgs/CompressedImage` and `sensor_msgs/CameraInfo`. We will run the node for 20-25 minutes without running any additional jobs on the Raspberry Pi and with `rqt` showing the streamed image and monitoring the two topics above. We will consider the load on Pi as the number reported by `htop` for the 15 minutes average load.

driver  | format | res [W x H]  | rate req [Hz] | rate real [Hz] | bandwidth [MBs] | load [proc]
------- | ------ | ------------ | ------------- | -------------- | --------------- | ------------
**usb_cam** |
usb_cam | mjpeg  | 160 x 120    | 20.0          | 30.0           | 0.135           | 0.08
usb_cam | mjpeg  | 176 x 144    | 20.0          | 30.0           | 0.144           | 0.15
usb_cam | mjpeg  | 320 x 240    | 20.0          | 30.0           | 0.326           | 0.21
usb_cam | mjpeg  | 352 x 288    | 20.0          | 30.0           | 0.386           | 0.25
usb_cam | mjpeg  | 640 x 480    | 20.0          | 30.0           | 0.895           | 0.85
usb_cam | mjpeg  | 1280 x 720   | 20.0          | 19.5           | 1.42            | 1.54
usb_cam | mjpeg  | 1980 x 1080  | 20.0          | 9.5            | 1.09            | 1.60
**cv_camera** |
cv_camera | mjpeg  | 160 x 120    | 30.0          | 30.0           | 0.143          | 0.21
cv_camera | mjpeg  | 640 x 480    | 20.0          | 20.0           | 0.55           | 0.50
cv_camera | mjpeg  | 1280 x 720   | 20.0          | 14.4           | 1.16           | 1.15

<!-- 
usb_cam | yuyv   | 176 x 144    | 20.0          | 13.0           | 0.065           | 0.10
usb_cam | yuyv   | 320 x 240    | 20.0          | 18.5           | 0.203           | 0.19
usb_cam | mjpeg  | 352 x 288    | 20.0          | 30.0           | 0.386           | 0.25


usb_cam | mjpeg  | 1980 x 1080  | 20.0          | 9.5            | 1.09            | 1.60 -->

`cv_camera` issues `Corrupt JPEG data: XX extraneous bytes before marker 0xd9`. To fix this issue in opencv: <https://github.com/opencv/opencv/issues/9477#issuecomment-805628550>. **Warning** takes a *very* long time to run!
