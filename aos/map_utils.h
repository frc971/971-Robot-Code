#ifndef AOS_MAP_UTILS_H_
#define AOS_MAP_UTILS_H_

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
// Returns true if the key was found and then populates *value with the value.
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

#endif  // AOS_MAP_UTILS_H_
