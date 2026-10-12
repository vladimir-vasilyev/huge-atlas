#pragma once

#include "chat/Json.h"
#include "chat/Tools.h"

#include <atomic>
#include <functional>
#include <string>

// The dialog with a model behind an OpenAI-compatible API (DeepSeek, OpenAI, Ollama, ...): the model answers or calls the tools of
// AtlasHost, the results go back, until it answers in words. `converse` blocks (run it on a thread of its own).
struct ChatEvents {
    std::function<void(const std::string& text)> assistant;                                  // the model said something
    std::function<void(const std::string& name, const Json& args)> toolCall;                 // the model calls a tool
    std::function<void(const std::string& name, const Json& result)> toolResult;
};

// One request: the JSON body in, the JSON answer out (false + message on a failure). The application passes the HTTP; tests pass a script.
using ChatTransport = std::function<bool(const std::string& body, std::string* response, std::string* error)>;

class ChatSession {
public:
    ChatSession() { reset(); }
    void reset();                       // forget the dialog
    size_t messageCount() const { return messages_.size(); }

    // Adds the user's message and runs the dialog to the model's final answer. false + message on a failure or a cancel ("cancelled").
    bool converse(const std::string& userText, const std::string& model, const ChatTransport& transport, AtlasHost& host,
                  const ChatEvents& events, const std::atomic<bool>* cancel, std::string* error, int maxSteps = 30);

private:
    Json messages_ = Json::array();
};
