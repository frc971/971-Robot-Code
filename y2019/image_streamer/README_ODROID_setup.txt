# To build and deploy the image_streamer code to the ODROID, run
#    ./deploy.sh 10.9.71.179
# This will also configure and copy a supervisor configuration
# file, vision.conf, to /etc/supervisor/conf.d.

# While the code can work with two cameras, as of March 4, 2019,
# the robot only has one driver camera on it.  To use
# two cameras, set --single_camera to false.  Use the
# exposure flag to set the camera exposure.

# To view a camera on a Debian laptop use:
mplayer -tv device=/dev/video0 tv://
# or if the video is not on video0 try
mplayer -tv device=/dev/video4 tv://
# Show available camera formats.
# This came from https://superuser.com/questions/494575/ffmpeg-open-webcam-using-yuyv-but-i-want-mjpeg
ffmpeg -f video4linux2 -list_formats all -i /dev/video2

# Jevois notes:
# To mount a jevois disk on a debian system, use y2019/vision/jevois-cmd
cd y2019/vision/
sudo ./jevois-cmd help
sudo ./jevois-cmd usbsd  # This will mount the disk.
# If needed, install python-serial to run jevois-cmd
sudo apt-get install python-serial

# Austin debugged the camera startup problems with supervisorctl on the ODROID.
# Here is a copy of how he did it.  The problem was the logging
# was not starting propery.  Austin added a flag to image_streamer
# so that it would not start logging by default.

root@odroid:~# supervisorctl
exposure_loop                    RUNNING   pid 625, uptime 0:09:09
vision                           FATAL     Exited too quickly (process log may have details)
supervisor> tail -1000 vision stderr
SURE_ABSOLUTE from 297 to 300
image_streamer(656)(00018): DEBUG   at 0000000021.963290s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_BRIGHTNESS from -3 to -40
image_streamer(656)(00019): DEBUG   at 0000000021.963585s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_GAIN from 16 to 0
image_streamer(656)(00020): INFO    at 0000000021.965102s: aos/vision/image/reader.cc: 277: Init: framerate ended up at 1/30
image_streamer(656)(00021): INFO    at 0000000021.967263s: aos/vision/image/reader.cc: 59: Reader: Bat Vision Successfully Initialized.
image_streamer(656)(00022): DEBUG   at 0000000021.969020s: aos/vision/image/reader.cc: 292: Start: queueing buffers for the first time
image_streamer(656)(00023): DEBUG   at 0000000021.969275s: aos/vision/image/reader.cc: 301: Start: done with first queue
image_streamer: y2019/vision/image_streamer.cc:60: BlobLog::BlobLog(const char *, const char *): Assertion `ofst_.is_open()' failed.

supervisor> restart vision
vision: ERROR (not running)
vision: ERROR (spawn error)
supervisor> status
exposure_loop                    RUNNING   pid 625, uptime 0:09:31
vision                           STARTING
supervisor> status
exposure_loop                    RUNNING   pid 625, uptime 0:09:33
vision                           BACKOFF   Exited too quickly (process log may have details)
supervisor> status
exposure_loop                    RUNNING   pid 625, uptime 0:09:34
vision                           BACKOFF   Exited too quickly (process log may have details)
supervisor> tail -1000 vision stderr
SURE_ABSOLUTE from 297 to 300
image_streamer(864)(00018): DEBUG   at 0000000590.870582s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_BRIGHTNESS from -3 to -40
image_streamer(864)(00019): DEBUG   at 0000000590.870856s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_GAIN from 16 to 0
image_streamer(864)(00020): INFO    at 0000000590.872400s: aos/vision/image/reader.cc: 277: Init: framerate ended up at 1/30
image_streamer(864)(00021): INFO    at 0000000590.874543s: aos/vision/image/reader.cc: 59: Reader: Bat Vision Successfully Initialized.
image_streamer(864)(00022): DEBUG   at 0000000590.876289s: aos/vision/image/reader.cc: 292: Start: queueing buffers for the first time
image_streamer(864)(00023): DEBUG   at 0000000590.876547s: aos/vision/image/reader.cc: 301: Start: done with first queue
image_streamer: y2019/vision/image_streamer.cc:60: BlobLog::BlobLog(const char *, const char *): Assertion `ofst_.is_open()' failed.

supervisor> tail -1000 vision stdout
SURE_ABSOLUTE from 297 to 300
image_streamer(864)(00018): DEBUG   at 0000000590.870582s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_BRIGHTNESS from -3 to -40
image_streamer(864)(00019): DEBUG   at 0000000590.870856s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_GAIN from 16 to 0
image_streamer(864)(00020): INFO    at 0000000590.872400s: aos/vision/image/reader.cc: 277: Init: framerate ended up at 1/30
image_streamer(864)(00021): INFO    at 0000000590.874543s: aos/vision/image/reader.cc: 59: Reader: Bat Vision Successfully Initialized.
image_streamer(864)(00022): DEBUG   at 0000000590.876289s: aos/vision/image/reader.cc: 292: Start: queueing buffers for the first time
image_streamer(864)(00023): DEBUG   at 0000000590.876547s: aos/vision/image/reader.cc: 301: Start: done with first queue
image_streamer: y2019/vision/image_streamer.cc:60: BlobLog::BlobLog(const char *, const char *): Assertion `ofst_.is_open()' failed.

root@odroid:~# /root/image_streamer --single_camera=true --camera0_exposure=600
image_streamer(890)(00000): INFO    at 0000000667.025668s: y2019/vision/image_streamer.cc: 298: main: In main.
image_streamer(890)(00001): INFO    at 0000000667.025722s: y2019/vision/image_streamer.cc: 300: main: before setting up tcp_server.
image_streamer(890)(00002): INFO    at 0000000667.025819s: aos/vision/events/tcp_server.cc: 76: SocketBindListenOnPort: connected to port: 80 on fd: 3
image_streamer(890)(00003): INFO    at 0000000667.025844s: y2019/vision/image_streamer.cc: 302: main: after setting up tcp_server.
image_streamer(890)(00004): INFO    at 0000000667.025872s: y2019/vision/image_streamer.cc: 303: main: In main.
image_streamer(890)(00005): INFO    at 0000000667.025896s: y2019/vision/image_streamer.cc: 305: main: In main.
image_streamer(890)(00006): INFO    at 0000000667.025913s: y2019/vision/image_streamer.cc: 307: main: In main.
image_streamer(890)(00007): INFO    at 0000000667.025933s: y2019/vision/image_streamer.cc: 309: main: In main.
image_streamer(890)(00008): INFO    at 0000000667.025952s: y2019/vision/image_streamer.cc: 311: main: In main.
image_streamer(890)(00009): INFO    at 0000000667.025968s: y2019/vision/image_streamer.cc: 314: main: In main.
image_streamer(890)(00010): INFO    at 0000000667.025985s: y2019/vision/image_streamer.cc: 317: main: In main.
image_streamer(890)(00011): INFO    at 0000000667.026001s: y2019/vision/image_streamer.cc: 319: main: In main.
image_streamer(890)(00012): INFO    at 0000000667.026017s: y2019/vision/image_streamer.cc: 322: main: In main.
Before setting up udp socket 5001
image_streamer(890)(00013): INFO    at 0000000667.026090s: y2019/vision/image_streamer.cc: 324: main: In main.
image_streamer(890)(00014): INFO    at 0000000667.026142s: y2019/vision/image_streamer.cc: 328: main: In main.
image_streamer(890)(00015): WARNING at 0000000667.026220s: aos/vision/image/reader.cc: 217: Init: xioctl VIDIOC_S_CROP due to 25 (Inappropriate ioctl for device)
image_streamer(890)(00016): DEBUG   at 0000000667.026646s: aos/vision/image/reader.cc: 162: SetCameraControl: Camera control V4L2_CID_EXPOSURE_AUTO was already 1
image_streamer(890)(00017): DEBUG   at 0000000667.027819s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_EXPOSURE_ABSOLUTE from 300 to 600
image_streamer(890)(00018): DEBUG   at 0000000667.027887s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_BRIGHTNESS from -3 to -40
image_streamer(890)(00019): DEBUG   at 0000000667.027956s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_GAIN from 16 to 0
image_streamer(890)(00020): INFO    at 0000000667.029905s: aos/vision/image/reader.cc: 277: Init: framerate ended up at 1/30
image_streamer(890)(00021): INFO    at 0000000667.031824s: aos/vision/image/reader.cc: 59: Reader: Bat Vision Successfully Initialized.
image_streamer(890)(00022): DEBUG   at 0000000667.033340s: aos/vision/image/reader.cc: 292: Start: queueing buffers for the first time
image_streamer(890)(00023): DEBUG   at 0000000667.033369s: aos/vision/image/reader.cc: 301: Start: done with first queue
Logging to file (./logging/blob_record_38.dat)
Running Camera
image_streamer(890)(00024): DEBUG   at 0000000667.236660s: aos/vision/image/reader.cc: 162: SetCameraControl: Camera control V4L2_CID_EXPOSURE_AUTO was already 1
image_streamer(890)(00025): DEBUG   at 0000000667.238058s: aos/vision/image/reader.cc: 175: SetCameraControl: Set camera control V4L2_CID_EXPOSURE_ABSOLUTE from 15 to 600
image_streamer(890)(00026): INFO    at 0000000667.238178s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 29161
image_streamer(890)(00027): INFO    at 0000000667.238205s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00028): INFO    at 0000000667.252786s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 29521
image_streamer(890)(00029): INFO    at 0000000667.252809s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00030): INFO    at 0000000667.272826s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 29559
image_streamer(890)(00031): INFO    at 0000000667.272848s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00032): INFO    at 0000000667.316659s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 29640
image_streamer(890)(00033): INFO    at 0000000667.316680s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00034): INFO    at 0000000667.377320s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 65435
image_streamer(890)(00035): INFO    at 0000000667.377346s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00036): INFO    at 0000000667.412857s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 65651
image_streamer(890)(00037): INFO    at 0000000667.412945s: y2019/vision/image_streamer.cc: 335: operator(): Got a frame cam0
image_streamer(890)(00038): INFO    at 0000000667.444955s: y2019/vision/image_streamer.cc: 278: ProcessImage: Data has length 65648

