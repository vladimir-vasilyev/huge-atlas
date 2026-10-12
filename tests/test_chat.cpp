// The chat core: JSON, the tools, and a dialog with a scripted model against a fake host.
#include "chat/Agent.h"
#include "chat/Json.h"
#include "chat/Tools.h"
#include "TestUtil.h"

#include <atomic>

namespace {

struct FakeHost : AtlasHost {
    std::vector<std::string> calls;
    Json openMesh(const std::string& path) override { calls.push_back("open " + path); Json j = Json::object(); j["faces"] = 100; return j; }
    Json makeAtlas(const Json& s) override
    {
        calls.push_back("make q=" + std::to_string(s.getInt("quality", -1)) + " tex=" + std::to_string(s.getInt("texture_size", -1)));
        Json j = Json::object();
        j["patches"] = 7;
        j["efficiency"] = 0.42;
        return j;
    }
    Json repack(int t, int m) override { calls.push_back("repack " + std::to_string(t) + " " + std::to_string(m)); return Json::object(); }
    Json state() override { Json j = Json::object(); j["open"] = true; return j; }
    Json layout() override { return Json::object(); }
    Json report() override { Json j = Json::object(); j["report"] = "text"; return j; }
    Json save(const std::string& f) override { calls.push_back("save " + f); return Json::object(); }
};

std::string toolCallAnswer(const std::string& id, const std::string& name, const std::string& argsJson)
{
    Json fn = Json::object();
    fn["name"] = name;
    fn["arguments"] = argsJson;
    Json call = Json::object();
    call["id"] = id;
    call["type"] = "function";
    call["function"] = fn;
    Json calls = Json::array();
    calls.push_back(call);
    Json msg = Json::object();
    msg["role"] = "assistant";
    msg["content"] = nullptr;
    msg["tool_calls"] = calls;
    Json choice = Json::object();
    choice["message"] = msg;
    Json choices = Json::array();
    choices.push_back(choice);
    Json out = Json::object();
    out["choices"] = choices;
    return out.dump();
}

std::string textAnswer(const std::string& text)
{
    Json msg = Json::object();
    msg["role"] = "assistant";
    msg["content"] = text;
    Json choice = Json::object();
    choice["message"] = msg;
    Json choices = Json::array();
    choices.push_back(choice);
    Json out = Json::object();
    out["choices"] = choices;
    return out.dump();
}

} // namespace

int main()
{
    // JSON: round trip, escapes, unicode, numbers, errors
    {
        Json j;
        std::string err;
        CHECK(Json::parse(R"({"a": [1, 2.5, -3e2, true, null], "s": "x\"y\né😀", "o": {"k": false}})", &j, &err));
        CHECK(j.find("a")->size() == 5 && j.find("a")->at(2).number() == -300.0);
        CHECK(j.getString("s") == "x\"y\n\xC3\xA9\xF0\x9F\x98\x80");
        CHECK(!j.find("o")->getBool("k", true));
        Json k;
        CHECK(Json::parse(j.dump(), &k, &err));
        CHECK(k.dump() == j.dump());
        CHECK(!Json::parse("{\"a\": }", &j, &err) && !err.empty());
        CHECK(!Json::parse("[1, 2] x", &j, &err));
        Json b = Json::object();
        b["n"] = 3;
        b["f"] = 0.5;
        b["t"] = "tab\t";
        CHECK(b.dump() == R"({"n":3,"f":0.5,"t":"tab\t"})");
    }
    // the tool specs are valid and complete
    {
        const Json specs = chatToolSpecs();
        CHECK(specs.size() == 7);
        Json back;
        CHECK(Json::parse(specs.dump(), &back, nullptr));
        for (const Json& t : specs.items()) CHECK(t.find("function")->find("parameters")->getString("type") == "object");
    }
    // file names stay in the output folder
    CHECK(chatSafeFileName("../../etc/passwd") == "passwd.obj");
    CHECK(chatSafeFileName("a/b/c.PLY") == "c.PLY");
    CHECK(chatSafeFileName("") == "atlas.obj");
    // a dialog: open, make with other settings, answer
    {
        FakeHost host;
        ChatSession session;
        int step = 0;
        std::vector<std::string> said, tools;
        ChatTransport transport = [&](const std::string& body, std::string* response, std::string*) {
            Json req;
            CHECK(Json::parse(body, &req, nullptr));
            CHECK(req.find("tools")->size() == 7 && req.getString("model") == "m");
            switch (step++) {
                case 0: *response = toolCallAnswer("c1", "get_state", "{}"); break;
                case 1: *response = toolCallAnswer("c2", "make_atlas", R"({"quality": 2, "texture_size": 4096})"); break;
                case 2: {
                    // the result of the tool went back
                    const Json& last = req.find("messages")->items().back();
                    CHECK(last.getString("role") == "tool" && last.getString("tool_call_id") == "c2");
                    CHECK(last.getString("content").find("0.42") != std::string::npos);
                    *response = textAnswer("efficiency is fine");
                    break;
                }
            }
            return true;
        };
        ChatEvents ev;
        ev.assistant = [&](const std::string& t) { said.push_back(t); };
        ev.toolCall = [&](const std::string& n, const Json&) { tools.push_back(n); };
        std::string err;
        CHECK(session.converse("make an atlas", "m", transport, host, ev, nullptr, &err));
        CHECK(step == 3 && said.size() == 1 && said[0] == "efficiency is fine");
        CHECK(tools.size() == 2 && tools[1] == "make_atlas");
        CHECK(host.calls.size() == 1 && host.calls[0] == "make q=2 tex=4096");
        CHECK(session.messageCount() == 1 + 1 + 2 + 2 + 1);   // system, user, (assistant, tool) x 2, assistant
    }
    // a failing provider, a cancel, a model that loops, bad arguments
    {
        FakeHost host;
        ChatSession session;
        std::string err;
        CHECK(!session.converse("x", "m", [](const std::string&, std::string*, std::string* e) { *e = "no network"; return false; }, host, {}, nullptr, &err) && err == "no network");
        CHECK(!session.converse("x", "m", [](const std::string&, std::string* r, std::string*) { *r = R"({"error": {"message": "bad key"}})"; return true; }, host, {}, nullptr, &err) && err == "bad key");
        std::atomic<bool> stop{true};
        CHECK(!session.converse("x", "m", [](const std::string&, std::string* r, std::string*) { *r = textAnswer("hi"); return true; }, host, {}, &stop, &err) && err == "cancelled");
        ChatSession loop;
        int n = 0;
        CHECK(!loop.converse("x", "m", [&](const std::string&, std::string* r, std::string*) { *r = toolCallAnswer("c" + std::to_string(n++), "get_state", "{}"); return true; }, host, {}, nullptr, &err, 5) && n == 5);
        ChatSession bad;
        int s2 = 0;
        CHECK(bad.converse("x", "m", [&](const std::string&, std::string* r, std::string*) { *r = s2++ == 0 ? toolCallAnswer("c", "save_atlas", "{not json") : textAnswer("ok"); return true; }, host, {}, nullptr, &err));
        CHECK(host.calls.empty());   // save_atlas without a file name was refused, nothing reached the host
    }
    return 0;
}
