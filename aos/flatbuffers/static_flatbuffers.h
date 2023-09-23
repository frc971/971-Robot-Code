#ifndef AOS_FLATBUFFERS_STATIC_FLATBUFFERS_H_
#define AOS_FLATBUFFERS_STATIC_FLATBUFFERS_H_
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/reflection_generated.h"
namespace aos::fbs {

// Raw C++ code needed to represent a single flatbuffer table.
// The various strings in this struct represent the actual C++ code that will be
// used for this object; it is split up into pieces in order to allow us to
// combine multiple flatbuffer tables into a single generated file (namely,
// pulling the include declarations out to the top and including a set of
// dependencies so that we can order the code correctly).
// Primarily exposed here to allow for testing of intermediate functions.
struct GeneratedObject {
  // Fully qualified name of the object, in flatbuffer schema rules (e.g.
  // aos.examples.Ping).
  std::string name;
  // All #include statements required for this object.
  std::set<std::string> include_declarations;
  // Fully qualified names of all sub-objects, in flatbuffer schema rules (e.g.
  // aos.examples.Ping). Used to manage ordering of codegen.
  std::set<std::string> subobjects;
  // Actual code specific to this object.
  std::string code;
};

// Produces generated code for all flatbuffer tables in the file corresponding
// to the provided Schema object.
std::string GenerateCodeForRootTableFile(const reflection::Schema *schema);

// Helper functions to generate the code for individual objects; primarily
// exposed for testing.
GeneratedObject GenerateCodeForObject(const reflection::Schema *schema,
                                      int object_index);
GeneratedObject GenerateCodeForObject(const reflection::Schema *schema,
                                      const reflection::Object *object);
}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_STATIC_FLATBUFFERS_H_
