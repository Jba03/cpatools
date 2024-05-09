#pragma once

#include <cpatools/types.hpp>

#include <iomanip>
#include <iterator>
#include <vector>
#include <sstream>
#include <unordered_map>

#pragma mark - Markup

namespace cpa::markup {

struct node {
  node() = default;
  node(const std::string& name) : _name(name) { /* ... */ }
  node(const std::string& name, const std::string& value) : _name(name), _value(value) { /* ... */ }
  
  auto name() const -> std::string { return _name; }
  auto value() const -> std::string { return _value; }
  auto begin() { return _children.begin(); }
  auto end() { return _children.end(); }
  auto add_child(const node& nd) -> node& { return _children.emplace_back(nd); }
  
  auto operator [](std::string name) -> node& {
    for (auto& node : _children) {
      if (node.name() == name) return node;
    }
    throw "No such node '" + name + "' found";
  }
  
  inline static auto serialize(node& nd, unsigned depth = 0) -> std::string {
    if (nd.name().length() == 0) {
      std::string text;
      for (auto child : nd) {
        text.append(serialize(child, depth));
      }
      return text;
    }
    
    std::string pad;
    for (int i = 0; i < depth * 2; i++)
      pad.append(" ");
    
    std::vector<std::string> lines;
    std::stringstream value(nd.value());
    for (std::string line; std::getline(value, line, '\n');) {
      lines.emplace_back(line);
    }
    
    std::string result;
    result.append(pad + nd.name());
    if (lines.size() == 1) {
      result.append(std::string(":") + " " + lines[0]);
    }
    result.append("\n");
    if (lines.size() > 1) {
      result.append("  ");
      for (std::string line : lines) {
        result.append(pad + std::string(":") + " " + line + "\n");
      }
    }
    for (auto child : nd) {
      result.append(serialize(child, depth + 1));
    }
    return result;
  }
  
  inline auto unserialize(std::string text) -> void {
    std::vector<std::string> lines;
    std::string line;
    std::stringstream s(text);
    while (std::getline(s, line, '\n')) {
      if (depth(line) != std::string::npos) {
        if (!line.substr(depth(line), std::string::npos).starts_with("//")) {
          lines.push_back(line);
        }
      }
    }
    
    int i = 0;
    while (i < lines.size()) {
      node node;
      node.parse(lines, i, " ");
      _children.emplace_back(node);
    }
  }
  
  inline auto depth(std::string text) -> size_t {
    return text.find_first_not_of(" \t\n\v\f\r");
  }
  
  auto valid(char c) const -> bool {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '.' || c == '-' || c == '_';
  }
  
  void parse(const std::vector<std::string>& text, int& i, std::string_view spacing) {
    const char *p = text[i++].c_str();
    _depth = depth(p);
    p += _depth;
    
    //name
    int length = 0;
    while (valid(p[length])) length++;
    if (length == 0) throw "Invalid node name";
    _name = std::string(p).substr(0, length);
    p += length;
    
    //value
    if (*p == ':') {
      int length = 1;
      while(p[length] && p[length] != '\n') length++;
      _value = std::string(p).substr(1, length - 1) + "\n";
      _value = _value.substr(spacing.length(), std::string::npos);
      p += length;
    }
    
    while (i < text.size()) {
      size_t subdepth = depth(text[i]);
      if (subdepth <= _depth) break;
      node node;
      node.parse(text, i, spacing);
      _children.push_back(node);
    }
  }
  
  auto children() {
    return _children;
  }
  
protected:
  size_t _depth;
  std::string _name;
  std::string _value;
  std::vector<node> _children;
};
  
}

#pragma mark - Serializer interface

namespace cpa {

/// Serializer & datatype representation converter
struct serializer {
  serializer() = default;
  enum Mode { Convert, Serialize } mode;
  
  /// Structure cross-platform block representation
  struct block {
    enum type {
      Integer,
      Real,
      Pointer,
    };
    
    template<typename T>
    void pointer(pointer<T>& v) {
      void* data = v;
      
    }
    
    union {
      uint64_t integer;
      float real;
      void* pointer;
    } data;
    
  private:
    std::vector<block> blocks;
  };
  
//  operator node&() {
//    return _root;
//  }
  
  struct node {
    enum type {
      Integer,
      Real,
      Pointer,
    };
    
    uint8_t* data = nullptr;
    size_t size;
  };
  
  template<typename T>
  void pointer(pointer<T>& v) {
    void* data = v;
    if (mode == Convert) {
      // pointer expansion
      //currentNode->
    }
  }
  
  template<typename T>
  void integer(T& v) {
    uint64_t i = uint64_t(v);
    //node nd(_serializer, std::to_string(i));
  }
  
  template<typename T>
  void real(T& v) {
    
  }
  
  template<typename T>
  void structure(T& v) {
    //_children.emplace_back(nd);
  }
  
  //_children.emplace_back(nd);
  std::unordered_map<memory::target_address_type, block*> pointers;
  
private:
  node* currentNode;
  
  //node _root { this, "root" };
};

}
