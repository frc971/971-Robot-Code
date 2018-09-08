# How to download the code to a robot if you have a working code computer

## Prerequisites
1. Have a computer that has already built the code. See //:README.md for
   directions on how to do this.

2. TODO: Physical setup instructions (ethernet cables, router, driver station).

3. TODO: Networking instructions (static ip address on ethernet without losing
   wireless networking, mDNS information).



## Downloading the code

1. Make sure that you can connect to the roboRIO.
```console
ssh admin@roboRIO-971-frc.local /bin/true
```

2. Download the code
```console
bazel run --cpu=roborio --compilation_mode=opt //y2018:download -- admin@roboRIO-971-frc.local
```

3. TODO: How to run the code using the driver station.

## Troubleshooting
* If step 1 fails, you probably are not connected properly to the roboRIO.
  Double check prerequisites 1 and 2.
