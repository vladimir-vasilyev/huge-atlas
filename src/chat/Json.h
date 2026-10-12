#pragma once

#include <string>
#include <utility>
#include <vector>

// A small JSON value for the chat with a language model: parse, build, dump. Objects keep the order of their keys.
class Json {
public:
    enum Type { Null, Bool, Number, String, Array, Object };

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool b) : type_(Bool), num_(b ? 1 : 0) {}
    Json(int n) : type_(Number), num_(n) {}
    Json(long n) : type_(Number), num_(double(n)) {}
    Json(long long n) : type_(Number), num_(double(n)) {}
    Json(unsigned n) : type_(Number), num_(n) {}
    Json(size_t n) : type_(Number), num_(double(n)) {}
    Json(double n) : type_(Number), num_(n) {}
    Json(const char* s) : type_(String), str_(s) {}
    Json(std::string s) : type_(String), str_(std::move(s)) {}

    static Json array() { Json j; j.type_ = Array; return j; }
    static Json object() { Json j; j.type_ = Object; return j; }

    Type type() const { return type_; }
    bool isNull() const { return type_ == Null; }
    bool isObject() const { return type_ == Object; }
    bool isArray() const { return type_ == Array; }
    bool isString() const { return type_ == String; }
    bool isNumber() const { return type_ == Number; }

    // objects
    Json& operator[](const std::string& key);                 // inserts null when missing (a null becomes an object)
    const Json* find(const std::string& key) const;           // null pointer when missing
    bool has(const std::string& key) const { return find(key) != nullptr; }
    std::string getString(const std::string& key, const std::string& fallback = "") const;
    double getNumber(const std::string& key, double fallback = 0.0) const;
    int getInt(const std::string& key, int fallback = 0) const { return int(getNumber(key, fallback)); }
    bool getBool(const std::string& key, bool fallback = false) const;
    const std::vector<std::pair<std::string, Json>>& members() const { return object_; }

    // arrays
    void push_back(Json v) { if (type_ == Null) type_ = Array; array_.push_back(std::move(v)); }
    size_t size() const { return type_ == Array ? array_.size() : type_ == Object ? object_.size() : 0; }
    const Json& at(size_t i) const { return array_[i]; }
    const std::vector<Json>& items() const { return array_; }

    // scalars
    const std::string& str() const { return str_; }
    double number() const { return num_; }
    bool boolean() const { return num_ != 0; }

    std::string dump() const;
    // false and a message when the text is not JSON
    static bool parse(const std::string& text, Json* out, std::string* error);

private:
    void dumpTo(std::string& out) const;

    Type type_ = Null;
    double num_ = 0;
    std::string str_;
    std::vector<Json> array_;
    std::vector<std::pair<std::string, Json>> object_;
};
