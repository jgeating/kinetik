#ifndef NLOHMANN_JSON_HPP
#define NLOHMANN_JSON_HPP

/**
 * @brief Minimal JSON implementation for simulation
 * 
 * This is a simplified JSON implementation to avoid dependency issues
 * with PlatformIO native platform. For production use, consider using
 * the full nlohmann/json library.
 */

#include <string>
#include <map>
#include <vector>
#include <variant>
#include <sstream>
#include <iostream>

namespace nlohmann {

class json {
public:
    using value_type = std::variant<
        std::nullptr_t,
        bool,
        int64_t,
        double,
        std::string,
        std::vector<json>,
        std::map<std::string, json>
    >;

private:
    value_type m_value;

public:
    // Constructors
    json() : m_value(nullptr) {}
    json(std::nullptr_t) : m_value(nullptr) {}
    json(bool b) : m_value(b) {}
    json(int i) : m_value(static_cast<int64_t>(i)) {}
    json(int64_t i) : m_value(i) {}
    json(double d) : m_value(d) {}
    json(const char* s) : m_value(std::string(s)) {}
    json(const std::string& s) : m_value(s) {}
    json(const std::vector<json>& arr) : m_value(arr) {}
    json(const std::map<std::string, json>& obj) : m_value(obj) {}

    // Type checking
    bool is_null() const { return std::holds_alternative<std::nullptr_t>(m_value); }
    bool is_boolean() const { return std::holds_alternative<bool>(m_value); }
    bool is_number_integer() const { return std::holds_alternative<int64_t>(m_value); }
    bool is_number_float() const { return std::holds_alternative<double>(m_value); }
    bool is_string() const { return std::holds_alternative<std::string>(m_value); }
    bool is_array() const { return std::holds_alternative<std::vector<json>>(m_value); }
    bool is_object() const { return std::holds_alternative<std::map<std::string, json>>(m_value); }

    // Value access
    bool get_bool() const { return std::get<bool>(m_value); }
    int64_t get_int() const { return std::get<int64_t>(m_value); }
    double get_double() const { return std::get<double>(m_value); }
    const std::string& get_string() const { return std::get<std::string>(m_value); }
    const std::vector<json>& get_array() const { return std::get<std::vector<json>>(m_value); }
    const std::map<std::string, json>& get_object() const { return std::get<std::map<std::string, json>>(m_value); }

    // Operators
    json& operator[](const std::string& key) {
        if (!is_object()) {
            m_value = std::map<std::string, json>();
        }
        return std::get<std::map<std::string, json>>(m_value)[key];
    }

    const json& operator[](const std::string& key) const {
        static json null_json;
        if (!is_object()) return null_json;
        auto& obj = std::get<std::map<std::string, json>>(m_value);
        auto it = obj.find(key);
        return (it != obj.end()) ? it->second : null_json;
    }

    json& operator[](size_t index) {
        if (!is_array()) {
            m_value = std::vector<json>();
        }
        auto& arr = std::get<std::vector<json>>(m_value);
        if (index >= arr.size()) {
            arr.resize(index + 1);
        }
        return arr[index];
    }

    // Conversion operators
    operator bool() const { return is_boolean() ? get_bool() : false; }
    operator int() const { return is_number_integer() ? static_cast<int>(get_int()) : 0; }
    operator double() const { 
        if (is_number_float()) return get_double();
        if (is_number_integer()) return static_cast<double>(get_int());
        return 0.0;
    }
    operator std::string() const { return is_string() ? get_string() : ""; }

    // Utility methods
    bool contains(const std::string& key) const {
        if (!is_object()) return false;
        auto& obj = std::get<std::map<std::string, json>>(m_value);
        return obj.find(key) != obj.end();
    }

    void push_back(const json& value) {
        if (!is_array()) {
            m_value = std::vector<json>();
        }
        std::get<std::vector<json>>(m_value).push_back(value);
    }

    size_t size() const {
        if (is_array()) return std::get<std::vector<json>>(m_value).size();
        if (is_object()) return std::get<std::map<std::string, json>>(m_value).size();
        return 0;
    }

    // Serialization (basic)
    std::string dump(int indent = -1) const {
        std::ostringstream oss;
        dump_to_stream(oss, indent, 0);
        return oss.str();
    }

private:
    void dump_to_stream(std::ostream& os, int indent, int current_indent) const {
        if (is_null()) {
            os << "null";
        } else if (is_boolean()) {
            os << (get_bool() ? "true" : "false");
        } else if (is_number_integer()) {
            os << get_int();
        } else if (is_number_float()) {
            os << get_double();
        } else if (is_string()) {
            os << "\"" << get_string() << "\"";
        } else if (is_array()) {
            os << "[";
            auto& arr = get_array();
            for (size_t i = 0; i < arr.size(); ++i) {
                if (i > 0) os << ",";
                if (indent >= 0) os << "\n" << std::string(current_indent + indent, ' ');
                arr[i].dump_to_stream(os, indent, current_indent + indent);
            }
            if (indent >= 0 && !arr.empty()) os << "\n" << std::string(current_indent, ' ');
            os << "]";
        } else if (is_object()) {
            os << "{";
            auto& obj = get_object();
            bool first = true;
            for (auto& [key, value] : obj) {
                if (!first) os << ",";
                first = false;
                if (indent >= 0) os << "\n" << std::string(current_indent + indent, ' ');
                os << "\"" << key << "\":";
                if (indent >= 0) os << " ";
                value.dump_to_stream(os, indent, current_indent + indent);
            }
            if (indent >= 0 && !obj.empty()) os << "\n" << std::string(current_indent, ' ');
            os << "}";
        }
    }
};

// Static parse method (basic implementation)
inline json parse(const std::string& str) {
    // This is a very basic parser - for production use a proper JSON parser
    json result;
    // For now, just return empty object
    result = std::map<std::string, json>();
    return result;
}

} // namespace nlohmann

#endif // NLOHMANN_JSON_HPP
