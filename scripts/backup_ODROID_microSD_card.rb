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
       Backup an ODROID microSD card.
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

# Get the partition uuid
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

# Get the size of the size of the Master Boot Record (MBR).
# Also include the 512 bytes for the partition table.
# The MBR resides in the space before start of the first
# partition.
def get_MBR_size(device)
  # Use Json to parse the sfdisk output.
  sfdisk = JSON.parse(`sfdisk --json /dev/#{device}`)
  first_partition_start_in_blocks = sfdisk['partitiontable']['partitions'][0]['start']
  sector_size_in_bytes = `cat /sys/block/#{device}/queue/hw_sector_size`.chomp.to_i
  return ([first_partition_start_in_blocks, sector_size_in_bytes])
end

$start_time = Time.now
$date=`date "+%Y%m%d.%H%M"`.chomp
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

# Locate the root partition.
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
puts HELP_MSG_SHORT
puts "\n# Backing up ODROID XU4 microSD card.\n"
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

# Ask the user for the name of the device to be cloned.
loop do
  printf "\n# Enter the name of the device to be cloned [#{non_root_devices}]: "
  clone_source_device = gets.chomp
  STDOUT.flush
  if devices.include?(clone_source_device) then
    break
  else
    puts "ERROR: device name '#{clone_source_device}' not found.  Please try again."
  end
end

puts "# Using #{clone_source_device} for the device to be cloned."

#
# Make tar and dd files of the microSD card.
#

printf("\n# Provide a comment to be included in the filenames. i.e. 971-champs: ")
comment = gets.chomp.gsub("_","-")
#comment = "971_champs".gsub("_","-")
puts "# Using comment: #{comment}"

# List the partitions
partition1, partition2 = get_partitions(clone_source_device)
partition1_label = get_label(partition1)
partition2_label = get_label(partition2)
partition1_uuid = get_uuid(partition1)
partition2_uuid = get_uuid(partition2)
puts "\n# Summary information:"
puts "# Partition 1 is #{partition1} with label #{partition1_label} and uuid #{partition1_uuid}"
puts "# Partition 2 is #{partition2} with label #{partition2_label} and uuid #{partition2_uuid}"

base_name = "#{$date}_ODROID_XU4"

tar_file_name_boot = "#{base_name}_p1_#{comment}_#{partition1_label}_#{partition1_uuid}.tgz"
tar_file_name_root = "#{base_name}_p2_#{comment}_#{partition2_label}_#{partition2_uuid}.tgz"
dd_file_name_mbr = "#{base_name}_MBR_#{comment}.dd"
sfdisk_file_name = "#{base_name}_sfdisk_#{comment}.txt"
puts "# Using disk partition information name: #{sfdisk_file_name}"
puts "# Using boot partition backup name: #{tar_file_name_boot }"
puts "# Using root partition backup name: #{tar_file_name_root}"
puts "# Using disk MBR (master boot record) information name: #{dd_file_name_mbr}"

puts "\n# Backing up the sfdisk partition information."
echo_and_run("sfdisk -d /dev/#{clone_source_device} > #{sfdisk_file_name}")

puts "\n# Backing up the boot partition."
`mkdir /new` if ! Dir.exists?("/new")
puts "# Make sure nothing is mounted on /new before mounting #{partition1} there."
puts "#   Here is df output to check to see if anything is mounted on /new."
echo_and_run("df /new")
puts "#   Running unmount /new to make sure nothing is mounted there."
echo_and_run("umount /new")
mount(partition1,"/new")
echo_and_run("#{$time} tar -czf #{tar_file_name_boot} -C /new .")
umount(partition1)

puts "\n# Backing up the root partition."
mount(partition2,"/new")
echo_and_run("#{$time} tar -czf #{tar_file_name_root} -C /new .")
umount(partition2)

puts "\n# Backing up the master boot record using dd"
mbr_size_in_blocks, sector_size_in_bytes = get_MBR_size(clone_source_device)
puts "#  The block size is #{sector_size_in_bytes} bytes and the master boot record is "
puts "#  #{mbr_size_in_blocks} blocks long for a total size of\
 #{mbr_size_in_blocks*sector_size_in_bytes} bytes."
echo_and_run("dd if=/dev/#{clone_source_device} of=#{dd_file_name_mbr} \
 bs=#{sector_size_in_bytes} count=#{mbr_size_in_blocks}")

$end_time = Time.now
puts ""
puts "Start time: #{$start_time}"
puts "  End time: #{$end_time}"
puts "======================================"
puts "Total time: #{time_to_minutes_and_seconds($end_time-$start_time)}"
puts ""
puts "# Done.\n"
