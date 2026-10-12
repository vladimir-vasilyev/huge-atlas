#pragma once

#include "chat/Agent.h"

#include <atomic>
#include <string>
#include <vector>

// Where the model is: an OpenAI-compatible endpoint. The key lives in the keychain of macOS, the rest in the user defaults.
struct ChatProviderPreset {
    const char* name;
    const char* baseUrl;
    const char* model;
    bool needsKey;
};
const std::vector<ChatProviderPreset>& chatProviderPresets();   // DeepSeek, OpenAI, Ollama (local), Custom

struct ChatConfig {
    std::string baseUrl = "https://api.deepseek.com";
    std::string model = "deepseek-chat";
    std::string apiKey;       // not stored in the defaults
    bool ready() const;       // the endpoint is set, and a key too unless the endpoint is local
};

ChatConfig loadChatConfig();                      // defaults + keychain
void saveChatConfig(const ChatConfig& config);    // the key goes to the keychain (an empty key removes it)

// POSTs the JSON body to <baseUrl>/chat/completions. Blocks; `cancel` stops it.
ChatTransport makeHttpTransport(const ChatConfig& config, const std::atomic<bool>* cancel);
