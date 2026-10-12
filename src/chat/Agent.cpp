#include "chat/Agent.h"

void ChatSession::reset()
{
    messages_ = Json::array();
    Json sys = Json::object();
    sys["role"] = "system";
    sys["content"] = chatSystemPrompt();
    messages_.push_back(std::move(sys));
}

bool ChatSession::converse(const std::string& userText, const std::string& model, const ChatTransport& transport, AtlasHost& host,
                           const ChatEvents& events, const std::atomic<bool>* cancel, std::string* error, int maxSteps)
{
    auto cancelled = [&] { return cancel && cancel->load(); };
    {
        Json u = Json::object();
        u["role"] = "user";
        u["content"] = userText;
        messages_.push_back(std::move(u));
    }
    const Json tools = chatToolSpecs();
    for (int step = 0; step < maxSteps; ++step) {
        if (cancelled()) { if (error) *error = "cancelled"; return false; }
        Json request = Json::object();
        request["model"] = model;
        request["messages"] = messages_;
        request["tools"] = tools;
        std::string response, err;
        if (!transport(request.dump(), &response, &err)) { if (error) *error = cancelled() ? "cancelled" : err; return false; }
        Json answer;
        if (!Json::parse(response, &answer, &err)) { if (error) *error = "the answer is not JSON: " + err; return false; }
        if (const Json* e = answer.find("error")) {
            if (error) *error = e->isObject() ? e->getString("message", "the provider refused the request") : "the provider refused the request";
            return false;
        }
        const Json* choices = answer.find("choices");
        if (!choices || !choices->isArray() || choices->size() == 0) { if (error) *error = "the answer has no choices"; return false; }
        const Json* message = choices->at(0).find("message");
        if (!message) { if (error) *error = "the answer has no message"; return false; }

        // the message goes back into the dialog as it came (tool_calls included), without null fields
        Json kept = Json::object();
        kept["role"] = "assistant";
        const Json* content = message->find("content");
        const std::string text = content && content->isString() ? content->str() : "";
        kept["content"] = text;
        const Json* calls = message->find("tool_calls");
        const bool hasCalls = calls && calls->isArray() && calls->size() > 0;
        if (hasCalls) kept["tool_calls"] = *calls;
        messages_.push_back(std::move(kept));
        if (!text.empty() && events.assistant) events.assistant(text);
        if (!hasCalls) return true;

        for (const Json& c : calls->items()) {
            const Json* fn = c.find("function");
            const std::string name = fn ? fn->getString("name") : "";
            Json args = Json::object();
            if (fn) {
                Json parsed;
                const std::string raw = fn->getString("arguments");
                if (raw.empty() || (Json::parse(raw, &parsed, nullptr) && parsed.isObject())) { if (!raw.empty()) args = parsed; }
            }
            if (events.toolCall) events.toolCall(name, args);
            Json result = cancelled() ? Json() : callTool(host, name, args);
            if (cancelled()) { result = Json::object(); result["error"] = "cancelled"; }
            if (events.toolResult) events.toolResult(name, result);
            Json t = Json::object();
            t["role"] = "tool";
            t["tool_call_id"] = c.getString("id");
            t["content"] = result.dump();
            messages_.push_back(std::move(t));
        }
        if (cancelled()) { if (error) *error = "cancelled"; return false; }
    }
    if (error) *error = "stopped: too many steps";
    return false;
}
