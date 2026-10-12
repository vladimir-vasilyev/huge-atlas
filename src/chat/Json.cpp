#include "chat/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

Json& Json::operator[](const std::string& key)
{
    if (type_ == Null) type_ = Object;
    for (auto& m : object_)
        if (m.first == key) return m.second;
    object_.emplace_back(key, Json());
    return object_.back().second;
}

const Json* Json::find(const std::string& key) const
{
    for (const auto& m : object_)
        if (m.first == key) return &m.second;
    return nullptr;
}

std::string Json::getString(const std::string& key, const std::string& fallback) const
{
    const Json* j = find(key);
    return j && j->type_ == String ? j->str_ : fallback;
}

double Json::getNumber(const std::string& key, double fallback) const
{
    const Json* j = find(key);
    if (!j) return fallback;
    if (j->type_ == Number || j->type_ == Bool) return j->num_;
    if (j->type_ == String) { char* end = nullptr; const double v = std::strtod(j->str_.c_str(), &end); return end != j->str_.c_str() ? v : fallback; }
    return fallback;
}

bool Json::getBool(const std::string& key, bool fallback) const
{
    const Json* j = find(key);
    if (!j) return fallback;
    if (j->type_ == Bool || j->type_ == Number) return j->num_ != 0;
    if (j->type_ == String) return j->str_ == "true" || j->str_ == "1";
    return fallback;
}

static void escapeTo(const std::string& s, std::string& out)
{
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) { char b[8]; std::snprintf(b, sizeof(b), "\\u%04x", c); out += b; }
                else out += char(c);
        }
    }
    out += '"';
}

void Json::dumpTo(std::string& out) const
{
    switch (type_) {
        case Null: out += "null"; break;
        case Bool: out += num_ != 0 ? "true" : "false"; break;
        case Number: {
            if (!std::isfinite(num_)) { out += "null"; break; }
            char b[40];
            if (num_ == std::floor(num_) && std::fabs(num_) < 1e15) std::snprintf(b, sizeof(b), "%lld", (long long)num_);
            else std::snprintf(b, sizeof(b), "%.10g", num_);
            out += b;
            break;
        }
        case String: escapeTo(str_, out); break;
        case Array:
            out += '[';
            for (size_t i = 0; i < array_.size(); ++i) { if (i) out += ','; array_[i].dumpTo(out); }
            out += ']';
            break;
        case Object:
            out += '{';
            for (size_t i = 0; i < object_.size(); ++i) {
                if (i) out += ',';
                escapeTo(object_[i].first, out);
                out += ':';
                object_[i].second.dumpTo(out);
            }
            out += '}';
            break;
    }
}

std::string Json::dump() const
{
    std::string out;
    dumpTo(out);
    return out;
}

namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;
    std::string error;

    explicit Parser(const std::string& text) : s(text) {}

    void ws() { while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i; }
    bool fail(const char* m) { if (error.empty()) error = std::string(m) + " at " + std::to_string(i); return false; }

    static void utf8(unsigned cp, std::string& out)
    {
        if (cp < 0x80) out += char(cp);
        else if (cp < 0x800) { out += char(0xC0 | (cp >> 6)); out += char(0x80 | (cp & 0x3F)); }
        else if (cp < 0x10000) { out += char(0xE0 | (cp >> 12)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
        else { out += char(0xF0 | (cp >> 18)); out += char(0x80 | ((cp >> 12) & 0x3F)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
    }

    bool hex4(unsigned& v)
    {
        if (i + 4 > s.size()) return fail("bad \\u escape");
        v = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = s[i++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= unsigned(c - '0');
            else if (c >= 'a' && c <= 'f') v |= unsigned(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= unsigned(c - 'A' + 10);
            else return fail("bad \\u escape");
        }
        return true;
    }

    bool string(std::string& out)
    {
        ++i;   // the opening quote
        while (i < s.size()) {
            const char c = s[i++];
            if (c == '"') return true;
            if (c != '\\') { out += c; continue; }
            if (i >= s.size()) break;
            const char e = s[i++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp;
                    if (!hex4(cp)) return false;
                    if (cp >= 0xD800 && cp < 0xDC00 && i + 1 < s.size() && s[i] == '\\' && s[i + 1] == 'u') {
                        i += 2;
                        unsigned lo;
                        if (!hex4(lo)) return false;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    utf8(cp, out);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }

    bool value(Json& out, int depth)
    {
        if (depth > 100) return fail("too deep");
        ws();
        if (i >= s.size()) return fail("unexpected end");
        const char c = s[i];
        if (c == '{') {
            ++i;
            out = Json::object();
            ws();
            if (i < s.size() && s[i] == '}') { ++i; return true; }
            for (;;) {
                ws();
                if (i >= s.size() || s[i] != '"') return fail("expected a key");
                std::string key;
                if (!string(key)) return false;
                ws();
                if (i >= s.size() || s[i] != ':') return fail("expected ':'");
                ++i;
                Json v;
                if (!value(v, depth + 1)) return false;
                out[key] = std::move(v);
                ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return true; }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            ++i;
            out = Json::array();
            ws();
            if (i < s.size() && s[i] == ']') { ++i; return true; }
            for (;;) {
                Json v;
                if (!value(v, depth + 1)) return false;
                out.push_back(std::move(v));
                ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') { std::string str; if (!string(str)) return false; out = Json(std::move(str)); return true; }
        if (s.compare(i, 4, "true") == 0) { i += 4; out = Json(true); return true; }
        if (s.compare(i, 5, "false") == 0) { i += 5; out = Json(false); return true; }
        if (s.compare(i, 4, "null") == 0) { i += 4; out = Json(); return true; }
        char* end = nullptr;
        const double v = std::strtod(s.c_str() + i, &end);
        if (end == s.c_str() + i) return fail("unexpected character");
        i = size_t(end - s.c_str());
        out = Json(v);
        return true;
    }
};

} // namespace

bool Json::parse(const std::string& text, Json* out, std::string* error)
{
    Parser p(text);
    Json v;
    if (!p.value(v, 0)) { if (error) *error = p.error; return false; }
    p.ws();
    if (p.i != text.size()) { if (error) *error = "extra text at " + std::to_string(p.i); return false; }
    *out = std::move(v);
    return true;
}
