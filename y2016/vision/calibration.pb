calibration {
  robot: "competition"
  calibration: {
    focal_length: 1.236070312
    center_center_dist: 0.27051
    center_center_skew: -16.65
    camera_y_skew: 20.0
    measure_camera_offset: 0.0
    camera_image_width: 1280
    camera_image_height: 960
    left_camera_name: "/dev/video0"
    right_camera_name: "/dev/video1"
    roborio_ip_addr: "10.9.71.168"
    jetson_ip_addr: "10.9.71.179"
    camera_exposure: 10
    camera_brightness: 128
    camera_gain: 0
    camera_fps: 20
  }
}

calibration {
  robot: "practice"
  calibration {
    focal_length: 1.236070312
    center_center_dist: 0.26829
    center_center_skew: 0.0
    camera_y_skew: 0.0
    measure_camera_offset: 0.0
    camera_image_width: 1280
    camera_image_height: 960
    left_camera_name: "/dev/video0"
    right_camera_name: "/dev/video1"
    roborio_ip_addr: "10.9.71.168"
    jetson_ip_addr: "10.9.71.179"
    camera_exposure: 10
    camera_brightness: 128
    camera_gain: 0
    camera_fps: 20
  }
}

calibration {
  robot: "stereo_rig"
  calibration {
    focal_length: 1.21515625
    center_center_dist: 0.26829
    center_center_skew: 39.11
    camera_y_skew: 0.0
    measure_camera_offset: 0.06985
    camera_image_width: 1280
    camera_image_height: 960
    left_camera_name: "/dev/v4l/by-id/usb-046d_0825_8C1756A0-video-index0"
    right_camera_name: "/dev/v4l/by-id/usb-046d_081a_E56C8BA0-video-index0"
    roborio_ip_addr: "10.9.71.168"
    jetson_ip_addr: "10.9.71.179"
    camera_exposure: 10
    camera_brightness: 128
    camera_gain: 0
    camera_fps: 20
  }
}
