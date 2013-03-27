#ifndef SIGNAL_HANDLER_H_
#define SIGNAL_HANDLER_H_

#include <boost/function.hpp>
#include <map>

// TODO(aschuh): Template std::map as well.
// Maps the key to the value, inserting it if it isn't there, or replacing it if
// it is.  Returns true if the key was added and false if it was replaced.
template <typename K, typename V>
bool InsertIntoMap(std::map<K, V> *my_map, const K &key, const V &new_value) {
  std::pair<typename std::map<K, V>::iterator, bool> element;
  element = my_map->insert(std::pair<K,V>(key, new_value));
  if (element.second == false) {
    element.first->second = new_value;
  }
  return element.second;
}

// Gets the value for the key from the map.
// Returns true if the key was found and then populates value with the value.
// Otherwise, leaves value alone and returns false.
template <typename K, typename V>
bool GetFromMap(const std::map<K, V> &my_map, const K &key, V *value) {
  typename std::map<K, V>::const_iterator element = my_map.find(key);
  if (element != my_map.end()) {
    *value = element->second;
    return true;
  }
  return false;
}


// Registers a signal handler to be run when we get the specified signal.
// The handler will run in a thread, rather than in the signal's context.
void RegisterSignalHandler(int signal_number,
                           boost::function<void(int)> function);

#endif  // SIGNAL_HANDLER_H_
