#ifndef FRC971_VISION_MEDIA_DEVICE_H_
#define FRC971_VISION_MEDIA_DEVICE_H_

#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "glog/logging.h"

#include "aos/scoped/scoped_fd.h"

namespace frc971 {
namespace vision {

class MediaDevice;
class Pad;
class Entity;

// See
// https://www.kernel.org/doc/html/v4.9/media/uapi/mediactl/media-controller.html
// for more info.
//
// This set of intertwined classes represents a Media device as exposed by the
// media controller API from V4L2.
//
// A MediaDevice is a peripheral.
//
// That piece of hardware has Entities which are functions that that peripheral
// exposes.  This can be things like various steps in the ISP, or
// encoder/decoder pipelines.
//
// Those entities each have Pads.  Pads are inputs or outputs from each piece of
// hardware.
//
// Pads are connected by Links.  Links can only be enabled/disabled, not
// connected at will (from what I can tell).
//
// The underlying API has enough info in it to make all this possible to deduce
// without classes, but given how much we want to manipulate things, it is
// easier to create a set of classes to index things better and add helper
// functions.

// A link between pads.
class Link {
 public:
  // Returns the source pad where the data comes from.
  Pad *source() { return source_; }
  const Pad *source() const { return source_; }
  // Returns the sink pad where the data goes to.
  Pad *sink() { return sink_; }
  const Pad *sink() const { return sink_; }

  // Returns true if this pad is enabled.
  bool enabled() const { return !!(flags_ & MEDIA_LNK_FL_ENABLED); }
  // Returns true if this pad can be enabled/disabled.
  bool immutable() const { return !!(flags_ & MEDIA_LNK_FL_IMMUTABLE); }
  // Returns true if this pad is dynamic.
  bool dynamic() const { return !!(flags_ & MEDIA_LNK_FL_DYNAMIC); }

  // Returns the raw flags.
  uint32_t flags() const { return flags_; }

 private:
  friend class MediaDevice;
  uint32_t flags_;
  uint32_t id_;

  // Pointers to this pad.  MediaDevice is responsible for updating the pointers
  // and maintaining their lifetime.
  Pad *source_ = nullptr;
  Pad *sink_ = nullptr;
};

// A pad of an entity connected by links.
class Pad {
 public:
  // Returns the unique ID that v4l2 assigns to this pad.
  uint32_t id() const { return id_; }
  // The pad index that some APIs need to talk about this pad.  This starts from
  // 0 and counts up.
  uint16_t index() const { return index_; }

  // The entity that this pad is on.
  const Entity *entity() const { return entity_; }

  // Returns true if this pad is a source.
  bool source() const { return !!(flags_ & MEDIA_PAD_FL_SOURCE); }
  // Returns true if this pad is a sink.
  bool sink() const { return !!(flags_ & MEDIA_PAD_FL_SINK); }

  // Returns the number of links.
  size_t links_size() const { return links_.size(); }
  // Returns a specific link.
  Link *links(size_t index) { return links_[index]; }
  const Link *links(size_t index) const { return links_[index]; }

  // Prints out a representation of this pad for debugging.
  void Log() const;

  // Sets the format of this pad.
  // TODO(austin): Should this set the paired sink pad automatically?  Formats
  // need to match...
  void SetSubdevFormat(uint32_t width, uint32_t height, uint32_t code);
  void SetSubdevCrop(uint32_t width, uint32_t height);

 private:
  friend class MediaDevice;

  uint16_t index_;

  uint32_t id_;
  uint32_t flags_;
  // List of links.  MediaDevice is responsible for updating them and their
  // lifetime.
  std::vector<Link *> links_;

  Entity *entity_ = nullptr;
};

// A hardware entity.
class Entity {
 public:
  // Returns the ID that v4l2 assignes to this entity.
  uint32_t id() const { return entity_.id; }
  // Returns the name of this entity.
  std::string_view name() const { return entity_.name; }

  // Returns the function that this entity serves.
  uint32_t function() const { return entity_.function; }
  // Returns the type of interface.
  uint32_t interface_type() const { return interface_.intf_type; }
  // Returns the major and minor device numbers.  Most likely, you just want the
  // device instead.
  uint32_t major() const { return interface_.devnode.major; }
  uint32_t minor() const { return interface_.devnode.minor; }

  // Returns the device needed to access this entity.
  const std::string &device() const {
    CHECK(has_interface_);
    return device_;
  }

  // Returns all the pads, in order.
  const std::vector<Pad *> pads() const { return pads_; }
  Pad *pads(size_t index) { return pads_[index]; }

  // Sets the format of this device.
  void SetFormat(uint32_t width, uint32_t height, uint32_t code);

  // Logs this device in a human readable format.
  void Log() const;

 private:
  friend class MediaDevice;

  // Updates device_.
  void UpdateDevice();

  struct media_v2_entity entity_;
  // If true, interface_ has been set.
  bool has_interface_ = false;
  struct media_v2_interface interface_ = {0, 0, 0, {}, {}};

  std::string device_;

  std::vector<Pad *> pads_;
};

// Class representing a media device.
class MediaDevice {
 public:
  static std::optional<MediaDevice> Initialize(int index);

  // Not copyable but movable.
  MediaDevice(const MediaDevice &) = delete;
  MediaDevice &operator=(const MediaDevice &) = delete;
  MediaDevice(MediaDevice &&) = default;
  MediaDevice &operator=(MediaDevice &&) = default;

  // Logs this media device in a human readable format.
  void Log() const;

  // Finds the entity with the provided name.
  Entity *FindEntity(std::string_view entity_name);

  // Finds the link connecting a source and destination entity through the
  // provided pads.  LOG(FATAL)'s if it can't find one.
  Link *FindLink(std::string_view source, int source_pad, std::string_view sink,
                 int sink_pad);

  // Disables all the mutable links in this device.
  void Reset();

  // TODO(austin): get the fd down deep enough into the link that this works.
  // Disables the link.
  void Reset(Link *link);
  // Enables the link.
  void Enable(Link *link);

  // Returns the name of the driver implementing the media API as a
  // NUL-terminated ASCII string. The driver version is stored in the
  // driver_version field.
  //
  // Driver specific applications can use this information to verify the driver
  // identity. It is also useful to work around known bugs, or to identify
  // drivers in error reports.
  std::string_view driver() const { return device_info_.driver; }

  // Returns the device model name as a NUL-terminated UTF-8 string. The device
  // version is stored in the device_version field and is not be appended to the
  // model name.
  std::string_view model() const { return device_info_.model; }

  // Returns the serial number as a NUL-terminated ASCII string.
  std::string_view serial() const { return device_info_.serial; }

  // Returns the location of the device in the system as a NUL-terminated ASCII
  // string. This includes the bus type name (PCI, USB, ...) and a bus-specific
  // identifier.
  std::string_view bus_info() const { return device_info_.bus_info; }

  // Returns the list of entities in this device.
  std::vector<Entity> *entities() { return &entities_; }

  // Returns all the links between pads in this device.
  std::vector<Link> *links() { return &links_; }

 private:
  MediaDevice(int fd) : fd_(fd) { Update(); }

  void Update();

  friend class Link;
  friend class Entity;
  friend class Pad;

  aos::ScopedFD fd_;

  std::vector<Pad> pads_;
  std::vector<Entity> entities_;
  std::vector<Link> links_;

  struct media_device_info device_info_;
};

// Returns the MediaDevice with the requested bus_info() if it can be found, or
// nullopt otherwise.
std::optional<MediaDevice> FindMediaDevice(std::string_view device);

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_MEDIA_DEVICE_H_
