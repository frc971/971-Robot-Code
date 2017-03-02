#ifndef AOS_COMMON_UTIL_GLOBAL_FACTORY_H_
#define AOS_COMMON_UTIL_GLOBAL_FACTORY_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

// // File Usage Description:
// class ExampleBaseClass { virtual ~ExampleBaseClass(); }
// class ExampleSubClass : public ExampleBaseClass {}
//
// // At namespace scope in header file:
// SETUP_FACTORY(ExampleBaseClass);
// // At namespace scope in cc file:
// REGISTER_SUBCLASS("ExampleSubClass", ExampleBaseClass, ExampleSubClass);
//
// // When you want an object of type "ExampleSubClass".
// std::unique_ptr<ExampleBaseClass> constructed_item =
//     ExampleBaseClassGlobalFactory::Get("ExampleSubClass")();

// Helper macro to set up a Factory Family for a particular type.
// Put this is the header file along-side the base class.
#define SETUP_FACTORY(BaseClass, ...) \
  using BaseClass##GlobalFactory =    \
      ::aos::GlobalFactory<BaseClass, ##__VA_ARGS__>

// Helper macro to set up a Factory for a subtype. For BaseClass
// This should happen in a .cc file not a header file to avoid multiple
// linkage.
#define REGISTER_SUBCLASS_BY_KEY(key, BaseClass, SubClass) \
  BaseClass##GlobalFactory::SubClassRegisterer<SubClass>   \
      register_for_##SubClass(key)

// Proxy to above but where SubClass name is the key.
#define REGISTER_SUBCLASS(BaseClass, SubClass) \
  REGISTER_SUBCLASS_BY_KEY(#SubClass, BaseClass, SubClass)

namespace aos {

// Maintains a static std::unordered_map<std::string, FactoryFunction> for
// BaseClass. Best to use with macros above.
template <typename BaseClass, typename... FactoryArgs>
class GlobalFactory {
 public:
  using FactoryFunction =
      std::function<std::unique_ptr<BaseClass>(FactoryArgs &&...)>;

  // Gets the factory function by named. This will return a null factory
  // std::function if the factory is not available, so one would be wise
  // to check this function before use.
  // It is an error to call this during static initialization.
  static const FactoryFunction &Get(const std::string &name) {
    const auto &map = *GetMap();
    auto item = map.find(name);
    if (item == map.end()) {
      static FactoryFunction null_create_fn;
      return null_create_fn;
    }
    return item->second;
  }

  // Installs a factory function for constructing SubClass
  // using name "name". It is an error not call this at namespace scope
  // through the REGISTER_SUBCLASS macro above.
  template <typename SubClass>
  class SubClassRegisterer {
   public:
    explicit SubClassRegisterer(const char *name) {
      (*GetMap())[name] = [](FactoryArgs &&... args) {
        return std::unique_ptr<BaseClass>(
            new SubClass(std::forward<FactoryArgs>(args)...));
      };
    }
  };

  // Fetch all factory functions.
  static const std::unordered_map<std::string, FactoryFunction> &GetAll() {
    return *GetMap();
  }

 private:
  // Actual map. (Protected by static from concurrent construction
  // if there is nothing registered at static linkage time).
  static std::unordered_map<std::string, FactoryFunction> *GetMap() {
    static std::unordered_map<std::string, FactoryFunction> map;
    return &map;
  }
};

}  // namespace aos

#endif  // AOS_COMMON_UTIL_GLOBAL_FACTORY_H_
