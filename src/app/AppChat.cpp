// The assistant of the ImGui build: a window with the conversation, the provider setup, and the adapter that lets the model drive App.
#include "app/App.h"

#include "chat/Agent.h"
#include "chat/Provider.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

struct Line {
    std::string role, text;
};

} // namespace

struct App::ChatState {
    std::mutex mutex;
    std::vector<Line> lines;
    bool scroll = false;
    ChatSession session;
    std::thread thread;
    std::atomic<bool> stop{false}, busy{false};
    ChatConfig config;
    bool configLoaded = false;
    bool setupOpen = false;
    int provider = 0;
    char input[1024] = "";
    char url[256] = "", model[128] = "", key[256] = "";
    bool autoSent = false, autoDone = false;

    void add(const std::string& role, const std::string& text)
    {
        std::lock_guard<std::mutex> lock(mutex);
        lines.push_back({role, text});
        scroll = true;
    }
};

class AssistantHost : public AtlasHost {
public:
    explicit AssistantHost(App* app) : app_(app) {}
    Json openMesh(const std::string& path) override { return app_->hostOpen(path); }
    Json makeAtlas(const Json& s) override { return app_->hostMakeAtlas(s); }
    Json repack(int size, int mip) override { return app_->hostRepack(size, mip); }
    Json state() override { return app_->hostState(); }
    Json layout() override { return app_->hostLayout(); }
    Json report() override { return app_->hostReport(); }
    Json save(const std::string& name) override { return app_->hostSave(name); }
private:
    App* app_;
};

void App::runHostTasks()
{
    for (;;) {
        std::function<void()> task;
        {
            std::lock_guard<std::mutex> lock(hostMutex_);
            if (hostTasks_.empty()) return;
            task = std::move(hostTasks_.front());
            hostTasks_.pop_front();
        }
        task();
    }
}

void App::shutdownAssistant()
{
    quitting_ = true;
    if (!chat_) return;
    chat_->stop = true;
    if (chat_->thread.joinable()) chat_->thread.join();   // the agent thread leaves its waits when quitting_ is set
}

bool App::assistantAutoReady()
{
    if (!std::getenv("VSA_AUTO_CHAT")) return true;
    return chat_ && chat_->autoDone;
}

static void startConversation(App* app, const std::shared_ptr<App::ChatState>& chatPtr, const std::string& text);

void App::drawAssistantWindow()
{
    if (!chat_) chat_ = std::make_shared<ChatState>();
    ChatState& c = *chat_;
    if (!c.configLoaded) {
        c.config = loadChatConfig();
        c.configLoaded = true;
        std::snprintf(c.url, sizeof(c.url), "%s", c.config.baseUrl.c_str());
        std::snprintf(c.model, sizeof(c.model), "%s", c.config.model.c_str());
        std::snprintf(c.key, sizeof(c.key), "%s", c.config.apiKey.c_str());
        c.provider = int(chatProviderPresets().size()) - 1;
        for (size_t i = 0; i < chatProviderPresets().size(); ++i)
            if (c.config.baseUrl == chatProviderPresets()[i].baseUrl) c.provider = int(i);
        c.add("note", "I can open a mesh, make the atlas, read the numbers and tune the settings. Only the numbers go to the model provider, never the mesh. "
                      "Press Set up to choose the provider.");
    }
    if (const char* msg = std::getenv("VSA_AUTO_CHAT")) {   // testing hook: send this message once the mesh is open (VSA_CHAT_URL = the provider)
        static int frames = 0;
        if (!c.autoSent && ++frames > 4) {
            c.autoSent = true;
            showAssistant_ = true;
            if (const char* u = std::getenv("VSA_CHAT_URL")) { c.config.baseUrl = u; c.config.model = "test"; }
            startConversation(this, chat_, msg);
        }
    }
    if (!showAssistant_) return;

    ImGui::SetNextWindowPos(ImVec2(float(ImGui::GetIO().DisplaySize.x) - 440.0f, 36.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(420, 560), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Assistant", &showAssistant_)) { ImGui::End(); return; }

    if (ImGui::Button(c.setupOpen ? "Hide setup" : "Set up...")) c.setupOpen = !c.setupOpen;
    if (c.setupOpen) {
        const auto& presets = chatProviderPresets();
        if (ImGui::BeginCombo("Provider", presets[size_t(c.provider)].name)) {
            for (size_t i = 0; i < presets.size(); ++i)
                if (ImGui::Selectable(presets[i].name, int(i) == c.provider)) {
                    c.provider = int(i);
                    if (presets[i].baseUrl[0]) {
                        std::snprintf(c.url, sizeof(c.url), "%s", presets[i].baseUrl);
                        std::snprintf(c.model, sizeof(c.model), "%s", presets[i].model);
                    }
                }
            ImGui::EndCombo();
        }
        ImGui::InputText("Address", c.url, sizeof(c.url));
        ImGui::InputText("Model", c.model, sizeof(c.model));
        ImGui::InputText("API key", c.key, sizeof(c.key), ImGuiInputTextFlags_Password);
        if (ImGui::Button("Save")) {
            c.config.baseUrl = c.url;
            c.config.model = c.model;
            c.config.apiKey = c.key;
            saveChatConfig(c.config);
            c.setupOpen = false;
            c.add("note", "The provider is saved (the key is in the macOS keychain).");
        }
        ImGui::TextDisabled("Any OpenAI-compatible service: DeepSeek, OpenAI, a local Ollama.");
    }
    ImGui::Separator();

    const float footer = ImGui::GetFrameHeightWithSpacing() + 6.0f;
    ImGui::BeginChild("##conversation", ImVec2(0, -footer), false);
    {
        std::lock_guard<std::mutex> lock(c.mutex);
        for (const Line& l : c.lines) {
            ImVec4 color = ImGui::GetStyleColorVec4(ImGuiCol_Text);
            if (l.role == "user") color = ImVec4(0.40f, 0.70f, 1.0f, 1.0f);
            else if (l.role == "tool" || l.role == "note") color = ImVec4(0.62f, 0.62f, 0.66f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(l.text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            ImGui::Spacing();
        }
        if (c.scroll) { ImGui::SetScrollHereY(1.0f); c.scroll = false; }
    }
    ImGui::EndChild();

    const bool busy = c.busy;
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(-70.0f);
    bool send = ImGui::InputTextWithHint("##message", "Ask the assistant", c.input, sizeof(c.input), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(busy ? "Stop" : "Send", ImVec2(-1, 0))) {
        if (busy) { c.stop = true; hostCancel(); }
        else send = true;
    }
    if (send && !busy && c.input[0]) {
        const std::string text = c.input;
        c.input[0] = 0;
        startConversation(this, chat_, text);
        ImGui::SetKeyboardFocusHere(-1);
    }
    ImGui::End();
}

static void startConversation(App* app, const std::shared_ptr<App::ChatState>& chatPtr, const std::string& text)
{
    App::ChatState& c = *chatPtr;
    if (c.busy) return;
    if (!c.config.ready()) {
        c.add("note", "Set up the model provider first: the address, the model and the key.");
        c.setupOpen = true;
        return;
    }
    c.add("user", text);
    if (c.thread.joinable()) c.thread.join();
    c.busy = true;
    c.stop = false;
    const ChatConfig config = c.config;
    auto state = chatPtr;
    c.thread = std::thread([app, state, config, text]() {
        AssistantHost host(app);
        ChatEvents ev;
        ev.assistant = [&](const std::string& t) { state->add("assistant", t); };
        ev.toolCall = [&](const std::string& name, const Json& args) {
            const std::string a = args.dump();
            state->add("tool", name + (a == "{}" ? "" : " " + a));
        };
        ev.toolResult = [&](const std::string&, const Json& r) { if (r.has("error")) state->add("tool", "-> " + r.getString("error")); };
        std::string err;
        const bool ok = state->session.converse(text, config.model, makeHttpTransport(config, &state->stop), host, ev, &state->stop, &err);
        if (!ok) state->add("note", err == "cancelled" ? "Stopped." : "The assistant stopped: " + err);
        if (std::getenv("VSA_AUTO_CHAT")) std::printf("AUTO CHAT: %s\n", ok ? "done" : ("failed: " + err).c_str());
        state->busy = false;
        state->autoDone = true;
    });
}
