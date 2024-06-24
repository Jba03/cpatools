#pragma once

#include <cpatools/types.hpp>

#include <iomanip>
#include <iterator>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <cpatools/json/single_include/nlohmann/json.hpp>

namespace cpa {

struct serializable;

struct serializer {
  enum Mode { Load, Save } mode = Save;
  serializer(Mode m) : mode(m) {}
  
  using pointer_map = std::unordered_map<memory::target_address_type, std::any>;
  
  struct node {
    node() = default;
    node(serializer* s) {
      serializer = s;
    }
    
    inline void type(std::string name) {
      data["$datatype"] = name;
    }
    
    template<typename T>
    inline void integer(std::string name, T& v) {
      data[name] = uint32_t(v);
    }
    
    template<typename T>
    inline void real(std::string name, T& v) {
      data[name] = float(v);
    }
    
    template<typename T> requires std::is_class_v<T>
    inline void structure(std::string name, T& v) {
      node child(serializer);
      v.serialize(child);
      data[name] = child.data;
    }
    
    // Pointer
    template<typename T>
    inline void pointer(std::string name, cpa::pointer<T>& v) {
      memory::target_address_type addr = v.pointeeAddress().effectiveAddress();
      pointer_map& pointers = serializer->pointers;
      auto it = pointers.find(addr);
      constexpr bool blank_ptr = std::is_same_v<T, cpa::address>;
      if (blank_ptr || it != pointers.end()) {
        data[name] = "pointer<" + std::to_string(addr) + ">";
      } else {
        // Avoid entering an endless loop
        pointers[addr] = 0;
        try {
          node child(serializer);
          data["$address"] = addr;
          if (v) if constexpr (!blank_ptr) v->serialize(child);
          data[name] = child.data;
        } catch (...) {
        }
      }
    }
    
    template<size_t size>
    inline void string(std::string name, cpa::string<size>& v) {
      node child(serializer);
      child.type("string<" + std::to_string(size) + ">");
      data[name] = (const char*)(v);
    }
    
    template<typename T>
    void array(std::string name, cpa::pointer<T>& v, int length) {
      node child(serializer);
//      for (auto i : range(length)) {
//        T value = v[i];
//        node child2(serializer);
//        if constexpr (std::is_base_of_v<cpa::serializable, T>) {
//          value.serialize(child2);
//        } else {
//          child2.data = (typename T::underlying_type)(v[i]);
//        }
//        child.data.push_back(child2.data);
//      }
      data[name] = child.data;
    }
    
    serializer* serializer;
    nlohmann::ordered_json data;
  };
  
  pointer_map pointers;
};

struct serializable {
  auto serialize(serializer::node&);
};

}
