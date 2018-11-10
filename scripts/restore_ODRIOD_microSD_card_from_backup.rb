#!/usr/bin/env ruby

HELP_MSG_SHORT = <<END
#
# Usage: #{__FILE__} [-h]
#
#    -h  Print this help message.
#
END

HELP_MSG_LONG = <<END

Summary:
       Restore a backup image to an ODROID microSD card.
#
# The backup_ODROID_microSD_card.rb and restore_ODRIOD_microSD_card_from_backup.rb
# scripts are used to backup ODROID microSD cards and create new ones.  They use
# tar, mkfs, and dd.  dd is only used for the master boot record.  The traditional
# way of copying a mircoSD card is to use dd to make a copy of the entire disk and then
# copy that back using dd.  This can take a long time - hours - and requires having
# enough space to save a file that is the size of the microSD card.  These scripts
# run in less than 5 minutes each and use only as much disk space as is require
# to hold the compressed contents of the microSD card.
#
# This https://wiki.odroid.com/odroid-xu4/software/partition_table
# page has some useful information on how hard disks are put together.
# It appears that the Ubuntu Partition Table Mainline U-boot (odroidxu4-v2017.05)
# section near the bottom of the page is how the ODROIDs are set up.  The
# Boot Sequence part at the bottom is also helpful to understand the boot sequence.
#
# The scripts prompt the user for the device name to be used for the source
# of the backup and as the target for the restore.  The root filesystem of
# the computer running these scripts is not allowed as an option.
#
# Note: If you have trouble booting the ODROID, make sure the small switch
#       around the corner from the HDMI port is set to microSD and not MMC.
#       If this does not work, try pushing and holding for a few seconds the
#       power button on top of the case.  It is located right by the two USB ports.
#       Once it is booted, the blue heartbeat light should slowly flash.
#       https://magazine.odroid.com/wp-content/uploads/odroid-xu4-user-manual.pdf
#       has more information on the ODROID.
END

HELP_MSG = HELP_MSG_LONG + HELP_MSG_SHORT

require 'json'
require 'pp'

# Process the command line arguments.
def parse_args()
  # If there are any command line arguments, print the help message.
  # This script will fail otherwise because the command line arguments
  # are passed to the gets commands below which is not what we want.
  #
  # Print the help message if there
  if ((ARGV[0] == "-h") || (ARGV.length > 0) )
    puts HELP_MSG
    exit!()
  end
end

# Get the partition label
def get_label(partition)
  return(`blkid -o export #{partition}  | egrep "^LABEL="`.gsub(/LABEL=/,"").chomp)
end

def get_uuid(partition)
  return(`blkid -o export #{partition}  | egrep "^UUID="`.gsub(/UUID=/,"").chomp)
end

# Get the two ODROID partition names.
def get_partitions(device)
  sfdisk = JSON.parse(`sfdisk --json /dev/#{device}`)
  partitions = sfdisk['partitiontable']['partitions']
  partition1 = partition2 = nil # return nil if the partition does not exist
  partition1 = partitions[0]['node'] if partitions.length > 0
  partition2 = partitions[1]['node'] if partitions.length > 1
  [partition1, partition2]
end

def umount_all_partitions(device)
  sfdisk = JSON.parse(`sfdisk --json /dev/#{device}`)
  partitions = sfdisk['partitiontable']['partitions']
  puts "#   Unmounting all partitions on device #{device}"
  partitions.each do |partition_info|
    partition = partition_info['node']
    if is_mounted(partition) then
       echo_and_run("umount #{partition}")
    else
      puts "#     #{partition} not mounted so no need to unmount it."
    end
  end
end

# Check to see if a partition is mounted.
def is_mounted(partition)
  `lsblk -no MOUNTPOINT #{partition} | wc -c`.chomp.to_i>1
end

def umount(partition)
  #puts `df #{partition}` if is_mounted(partition)
  echo_and_run("umount #{partition}") if is_mounted(partition)
end

def mount(partition, dir)
  # First make sure it is not mounted.
  umount(partition)
  echo_and_run("mount #{partition} #{dir}")
end

# Echo and run a command.  Print out the output from the command too.
def echo_and_run(cmd)
  puts cmd
  puts `#{cmd}`
end

# Convert a float to minutes and seconds.
def time_to_minutes_and_seconds(time)
  min = time.to_i/60
  sec = time.to_i%60
  sprintf("%d:%02d minutes:seconds",min,sec)
end

$start_time = Time.now
$time = "/usr/bin/time -p"

parse_args()

# First figure out what devices are available.  Do not show the root filesystem as an option.
#
# lsblk gives a listing (shown below) of the devices and mount points.
# findmnt -n -o SOURCE / tells which device the root filesystem is mounted on.
# /dev/nvme0n1p6 for the file systems below.
#
# NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
# mmcblk0     179:0    0  29.7G  0 disk
# ├─mmcblk0p1 179:1    0   128M  0 part /media/michael/boot
# └─mmcblk0p2 179:2    0  14.8G  0 part /media/michael/rootfs
# nvme0n1     259:0    0 238.5G  0 disk
# ├─nvme0n1p1 259:1    0  1000M  0 part
# ├─nvme0n1p2 259:2    0   260M  0 part /boot/efi
# ├─nvme0n1p3 259:3    0   128M  0 part
# ├─nvme0n1p4 259:4    0 112.3G  0 part
# ├─nvme0n1p5 259:5    0  15.6G  0 part
# ├─nvme0n1p6 259:6    0  93.4G  0 part /
# └─nvme0n1p7 259:7    0  15.8G  0 part [SWAP]

root_partition = `findmnt -n -o SOURCE /`.chomp.gsub('/dev/',"")

# Use Json to parse the lsblk output.
lsblk = JSON.parse(`lsblk --json`)

# Create a list of devices
devices = []
partitions = {}

lsblk['blockdevices'].each { |device|
  devices.push(device_name = device["name"])
  # For each partition of this device, create a mapping from the partition to the device name.
  device['children'].each { |partition|
    partitions[partition["name"]]=device_name
  }
}
puts "\n# Restoring ODROID XU4 microSD card from a backup.\n"
puts "\n# The available devices are: #{devices.join(" ")}"
puts "# The root partition is: #{root_partition}"
puts "# The root device is #{partitions[root_partition]}.  This will not be offered as an option. "
# Remove the root partition device from the list of devices.
devices.delete(partitions[root_partition])
non_root_devices = devices.join(" ")

clone_source_device = ""

puts "# The non root devices are: #{non_root_devices}"
puts `lsblk #{non_root_devices.gsub(/\w+/){|s|'/dev/'+s}}`

if (non_root_devices.length == 0) then
  puts "\nERROR: No possible microSD card devices found.  Exiting.\n\n"
  exit -1
end

# Ask the user for the name of the device to be used for restoring the backup.
loop do
  printf "\n# Enter the name of the device to be overwritten with the backup [#{non_root_devices}]: "
  clone_source_device = gets.chomp
  STDOUT.flush
  if devices.include?(clone_source_device) then
    break
  else
    puts "ERROR: device name '#{clone_source_device}' not found.  Please try again."
  end
end

puts "# Using \"#{clone_source_device}\" for the device to be overwritten with the backup."

#
# Make tar and dd files of the microSD card.
#


puts "\n# Here is a list of backups available for the restore:"
puts `ls -ltr *_ODROID_XU4_MBR_*.dd`
date = ""
# Ask the user for the name of the device to be used for restoring the backup.
loop do
  printf "\n# Enter the backup date (i.e. 20181103.1111) from the list above for"
  printf "\n#   the restore source: "
  date = gets.chomp
  STDOUT.flush
  if (`ls #{date}_ODROID_XU4_MBR*.dd`.chomp.length > 10) then
    break
  else
    puts "ERROR: Backup with date '#{date}' not found.  Please try again."
  end
end

puts "\n# Using #{clone_source_device} for the device to be overwritten with the backup."
puts `lsblk /dev/#{clone_source_device}`
printf "\n# Enter the backup date (i.e. 181102.2224) for the archive to be overwritten with the backup: "
response = ""
loop do
  printf "\n# Are you sure!!  All data will be destroyed [Yes|No]: "
  response = gets.chomp
  STDOUT.flush
  exit if response.match("^[nN]o$")
  if (response.match("^[yY]es$")) then
    break
  else
    puts "ERROR: Enter 'Yes' or 'no'.  Please try again."
  end
end

sfdisk_file_name = `ls #{date}_ODROID_XU4_sfdisk_*.txt`.chomp
tar_file_name_boot = `ls #{date}_ODROID_XU4_p1_*.tgz`.chomp
tar_file_name_root = `ls #{date}_ODROID_XU4_p2_*.tgz`.chomp
dd_file_name_mbr = `ls #{date}_ODROID_XU4_MBR_*.dd`.chomp
puts "\n# Using backup files:"
puts `ls -l #{sfdisk_file_name} \
#{tar_file_name_boot} \
#{tar_file_name_root} \
#{dd_file_name_mbr}`

# Partition the microSD card
puts "\n# Partitioning the microSD card with device #{clone_source_device}"
puts "#"
puts "# First, make sure the partitions are not mounted."
umount_all_partitions(clone_source_device)
puts "#"
puts "# Partioning the disk using sfdisk #{sfdisk_file_name}"
echo_and_run("sfdisk /dev/#{clone_source_device} < #{sfdisk_file_name}")
puts "# Done partioning the disk using sfdisk #{sfdisk_file_name}"
puts "\n# Run partprobe so that the OS reads the new partition table to deal with above warning."
puts "partprobe"
`partprobe`

# Get the names of the two newly created partitions.
partition1, partition2 = get_partitions(clone_source_device)

puts "\n# Making new files systems"
a,b,c,d,e,label,uuid = tar_file_name_boot.gsub(/.tgz$/,"").split("_")
# Set the uuid with the -i flag. i.e. "mkfs.vfat -n boot -i 52AA6867 #{partition1}"
puts "#\n# Making boot partition with label '#{label}' with:"
echo_and_run("mkfs.vfat -n #{label} #{partition1}")

a,b,c,d,e,label,uuid = tar_file_name_root.gsub(/.tgz$/,"").split("_")
puts "#\n# Making root partition with label '#{label}' and uuid '#{uuid}' with"
puts "# The -F flag enables the force option."
echo_and_run("mkfs.ext4 -F -L #{label} -U #{uuid} #{partition2}")

puts "\n# Restoring backup to the boot partition."
`mkdir /new` if ! Dir.exists?("/new")
puts "# Make sure nothing is mounted on /new before mounting #{partition1} there."
puts "#   Here is df output to check to see if anything is mounted on /new."
echo_and_run("df /new")
puts "#   Running unmount /new to make sure nothing is mounted there."
echo_and_run("umount /new")
mount(partition1,"/new")
puts "# Using restore command:"
echo_and_run("#{$time} tar --numeric-owner -xzf #{tar_file_name_boot} -C /new")
umount(partition1)

puts "\n# Restoring backup to the root partition."
mount(partition2,"/new")
puts "# Using restore command:"
echo_and_run("#{$time} tar --numeric-owner -xzf #{tar_file_name_root} -C /new .")
puts "# Sync the disks to flush writing the tar file."
echo_and_run("#{$time} sync")
umount(partition2)

# On helium, I backed up the Master Boot Record, bootloader, and other
# header files with:root[533] helium /backups/linux/helium
# root@helium:# dd if=/dev/mmcblk1 of=/backups/linux/helium/20180902_helium_MBR.dd bs=512 count=8192
# dd if=${DISK} of=${ODROID_DIR}/20180923_ODROID_XU4_9971_MBR.dd bs=512 count=2048

# http://wiki.linuxquestions.org/wiki/Some_dd_examples
# shows how to copy one disk to another while skipping the partition table.
#    seek=N skip N obs-sized blocks at start of output
#    skip=N skip N ibs-sized blocks at start of input


# I wrote this with
# root@gold:/tmp# dd if=20180902_helium_MBR.dd of=/dev/mmcblk0 bs=512
#dd if=${ODROID_DIR}/20180923_ODROID_XU4_9971_MBR.dd skip=1 of=${DISK} seek=1 bs=512

puts "\n# Restoring MBR (master boot record) to the beginning of the device."
puts `ls -l #{dd_file_name_mbr}`
puts "# Using restore command:"
echo_and_run("#{$time} dd if=#{dd_file_name_mbr} skip=1 of=/dev/#{clone_source_device} seek=1 bs=512")

$end_time = Time.now
puts ""
puts "Start time: #{$start_time}"
puts "  End time: #{$end_time}"
puts "======================================"
puts "Total time: #{time_to_minutes_and_seconds($end_time-$start_time)}"
puts ""
puts "# Done.\n"
