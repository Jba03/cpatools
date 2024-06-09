#pragma once

#include <cpatools/types.hpp>

#include <iomanip>
#include <iterator>
#include <vector>
#include <sstream>
#include <unordered_map>

#include <cpatools/json.hpp>

namespace cpa {

struct serializable;

/// Serializer & datatype representation converter
struct serializer {
  enum Mode { Load, Save } mode = Save;
  serializer(Mode m) : mode(m) {}
  
  using pointer_map = std::unordered_map<memory::target_address_type, std::any>;
  
  struct node {
    node() = default;
    node(serializer* s) {
      serializer = s;
    }
    
    void type(std::string name) {
      data["$datatype"] = name;
    }
    
    template<typename T>
    void integer(std::string name, T& v) {
      //printf("add %s\n", name.c_str());
      data[name] = uint32_t(v);
    }
    
    template<typename T>
    void real(std::string name, T& v) {
      //printf("add %s\n", name.c_str());
      data[name] = float(v);
    }
    
    template<typename T> requires std::is_class_v<T>
    void structure(std::string name, T& v) {
      node child(serializer);
      v.serialize(child);
      data[name] = child.data;
    }
    
    // Blank pointer
    void pointer(std::string name, cpa::pointer<>& v) {
      memory::target_address_type addr = v.pointeeAddress().effectiveAddress();
      pointer_map& pointers = serializer->pointers;
      if (pointers.find(addr) != pointers.end()) {
//          node nd(serializer);
//          _children.emplace_back(nd);
      } else {
        try {
          node child(serializer);
          data[name] = child.data;
        } catch (...) {
        }
      
      }
    }
    
    // Pointer
    template<typename T>
    void pointer(std::string name, cpa::pointer<T>& v) {
      memory::target_address_type addr = v.pointeeAddress().effectiveAddress();
      pointer_map& pointers = serializer->pointers;
      auto it = pointers.find(addr);
      if (it != pointers.end()) {
//          node nd(serializer);
//          _children.emplace_back(nd);
      } else {
        // Avoid entering an endless loop
        pointers[addr] = 0;
        try {
          node child(serializer);
          data["$address"] = addr;
          if (v) v->serialize(child);
          data[name] = child.data;
        } catch (...) {
        }
      
      }
    }
    
    template<size_t size>
    void string(std::string name, cpa::string<size>& v) {
      //printf("add string: %s\n", v);
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
    
//    operator nlohmann::json& () const {
//      return *this;
//    }
    
    serializer* serializer;
    nlohmann::ordered_json data;
  };
  
//  struct node : markup::node {
//    node(serializer* s) : _serializer(s) {}
//
//    node(serializer* s, std::string name, std::string value = "") {
//      _name = name;
//      _value = value;
//      _serializer = s;
//    }
//
//    template<typename T>
//    void pointer(std::string name, pointer<T>& v) {
//      if (_serializer->mode == Save) {
//        memory::target_address_type addr = v.pointeeAddress().effectiveAddress();
//        pointer_map& pointers = _serializer->pointers;
//        if (pointers.find(addr) != pointers.end()) {
//          node nd(_serializer, name, "@" + std::to_string(addr));
//          _children.emplace_back(nd);
//        } else {
//          try {
//            pointers[addr] = v;
//            node nd(_serializer, name, std::to_string(addr));
//            nd.add_child(node(_serializer, "absoluteAddress", std::to_string(addr)));
//            v->serialize(nd);
//            _children.emplace_back(nd);
//          } catch (...) {
//
//          }
//        }
//      } else if (_serializer->mode == Load) {
//        printf("loading pointer\n");
//        //children().erase(children().begin());
//      }
//    }
//
//    template<typename T>
//    void integer(std::string name, T& v) {
//      if (_serializer->mode == Save) {
//        data[name] = uint64_t(v);
//      }
//
//      data[name] =
//
////      std::string typeName = "";
////      if constexpr (std::is_same_v<T, char8>) typeName = "char8";
////      if constexpr (std::is_same_v<T, uchar8>) typeName = "uchar8";
////      if constexpr (std::is_same_v<T, int8>) typeName = "int8";
////      if constexpr (std::is_same_v<T, uint8>) typeName = "uint8";
////      if constexpr (std::is_same_v<T, int16>) typeName = "int16";
////      if constexpr (std::is_same_v<T, uint16>) typeName = "uint16";
////      if constexpr (std::is_same_v<T, int32>) typeName = "int32";
////      if constexpr (std::is_same_v<T, uint32>) typeName = "uint32";
////      if constexpr (std::is_same_v<T, int64>) typeName = "int64";
////      if constexpr (std::is_same_v<T, uint64>) typeName = "uint64";
////      if constexpr (std::is_same_v<T, float32>) typeName = "float32"; //preferably avoided
////
////      json data;
////      data["type"] = ""
//
////      if (_serializer->mode == Save) {
////        node nd(_serializer, name, typeName + "(" + std::to_string(v) + ")");
////        add_child(nd);
////      } else if (_serializer->mode == Load) {
////        //children().erase(children().begin());
////
////        printf("loading integer: %s %s\n", getName().c_str(), getValue().c_str());
////        printf("loading integer: %s %s\n", children().front().children().front().getName().c_str(), children().front().children().front().getValue().c_str());
////        if (getValue().starts_with(typeName)) {
////          std::string after = getValue().substr(typeName.length());
////          after.pop_back();
////
////          printf("got integer: %s\n", after.c_str());
////        }
////
////        //children().front();
////
////      }
//    }
//
//    template<typename T>
//    void real(std::string name, T& v) {
////      if (_serializer->mode == Save) {
////        char buf[64];
////        std::sprintf(buf, "%gf", float(v));
////        node nd(_serializer, name, std::string(buf));
////        add_child(nd);
////      } else if (_serializer->mode == Load) {
////        printf("loading real: %s\n", getValue().c_str());
////        //children().erase(children().begin());
////      }
//    }
//
//    template<typename T>
//    void structure(std::string name, T& v) {
////      if (_serializer->mode == Save) {
////        node nd(_serializer, name);
////        v.serialize(nd);
////        add_child(nd);
////      } else if (_serializer->mode == Load) {
////        //children().erase(children().begin());
////      }
//    }
//
//    void type(std::string type) {
////      if (_serializer->mode == Save) {
////        _value = type;
////      } else if (_serializer->mode == Load) {
////        if (_value != type) fprintf(stderr, "serialization: structure types %s and %s do not match\n", _value.c_str(), type.c_str());
////      }
//    }
//
//    json data;
//
//    std::vector<node> _children;
//    serializer* _serializer;
//  };
  
  pointer_map pointers;
};

struct serializable {
  auto serialize(serializer::node&);
};

}
