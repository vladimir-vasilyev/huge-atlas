#import <Foundation/Foundation.h>
#import <Security/Security.h>

#include "chat/Provider.h"

static NSString* const kService = @"huge-atlas assistant";
static NSString* const kBaseUrl = @"assistantBaseURL";
static NSString* const kModel = @"assistantModel";

const std::vector<ChatProviderPreset>& chatProviderPresets()
{
    static const std::vector<ChatProviderPreset> presets = {
        {"DeepSeek", "https://api.deepseek.com", "deepseek-chat", true},
        {"OpenAI", "https://api.openai.com/v1", "gpt-4o-mini", true},
        {"Ollama (local)", "http://localhost:11434/v1", "qwen2.5", false},
        {"Custom", "", "", true},
    };
    return presets;
}

static bool isLocal(const std::string& url)
{
    return url.find("://localhost") != std::string::npos || url.find("://127.0.0.1") != std::string::npos || url.find("://[::1]") != std::string::npos;
}

bool ChatConfig::ready() const
{
    return !baseUrl.empty() && !model.empty() && (!apiKey.empty() || isLocal(baseUrl));
}

static NSDictionary* keyQuery()
{
    return @{(__bridge id)kSecClass: (__bridge id)kSecClassGenericPassword, (__bridge id)kSecAttrService: kService, (__bridge id)kSecAttrAccount: @"api key"};
}

ChatConfig loadChatConfig()
{
    ChatConfig c;
    NSUserDefaults* u = NSUserDefaults.standardUserDefaults;
    if (NSString* s = [u stringForKey:kBaseUrl]) c.baseUrl = s.UTF8String;
    if (NSString* s = [u stringForKey:kModel]) c.model = s.UTF8String;
    NSMutableDictionary* q = [keyQuery() mutableCopy];
    q[(__bridge id)kSecReturnData] = @YES;
    q[(__bridge id)kSecMatchLimit] = (__bridge id)kSecMatchLimitOne;
    CFTypeRef out = nullptr;
    if (SecItemCopyMatching((__bridge CFDictionaryRef)q, &out) == errSecSuccess && out) {
        NSData* d = (__bridge_transfer NSData*)out;
        NSString* s = [[NSString alloc] initWithData:d encoding:NSUTF8StringEncoding];
        if (s) c.apiKey = s.UTF8String;
    }
    return c;
}

void saveChatConfig(const ChatConfig& c)
{
    NSUserDefaults* u = NSUserDefaults.standardUserDefaults;
    [u setObject:[NSString stringWithUTF8String:c.baseUrl.c_str()] forKey:kBaseUrl];
    [u setObject:[NSString stringWithUTF8String:c.model.c_str()] forKey:kModel];
    SecItemDelete((__bridge CFDictionaryRef)keyQuery());
    if (!c.apiKey.empty()) {
        NSMutableDictionary* q = [keyQuery() mutableCopy];
        q[(__bridge id)kSecValueData] = [NSData dataWithBytes:c.apiKey.data() length:c.apiKey.size()];
        SecItemAdd((__bridge CFDictionaryRef)q, nullptr);
    }
}

ChatTransport makeHttpTransport(const ChatConfig& config, const std::atomic<bool>* cancel)
{
    return [config, cancel](const std::string& body, std::string* response, std::string* error) -> bool {
        std::string base = config.baseUrl;
        while (!base.empty() && base.back() == '/') base.pop_back();
        NSURL* url = [NSURL URLWithString:[NSString stringWithUTF8String:(base + "/chat/completions").c_str()]];
        if (!url) { if (error) *error = "the address of the provider is not valid"; return false; }
        NSMutableURLRequest* req = [NSMutableURLRequest requestWithURL:url];
        req.HTTPMethod = @"POST";
        req.timeoutInterval = 300;
        [req setValue:@"application/json" forHTTPHeaderField:@"Content-Type"];
        if (!config.apiKey.empty()) [req setValue:[NSString stringWithFormat:@"Bearer %s", config.apiKey.c_str()] forHTTPHeaderField:@"Authorization"];
        req.HTTPBody = [NSData dataWithBytes:body.data() length:body.size()];

        __block NSData* data = nil;
        __block NSError* err = nil;
        __block NSInteger status = 0;
        dispatch_semaphore_t done = dispatch_semaphore_create(0);
        NSURLSessionDataTask* task = [NSURLSession.sharedSession dataTaskWithRequest:req completionHandler:^(NSData* d, NSURLResponse* r, NSError* e) {
            data = d;
            err = e;
            status = [r isKindOfClass:NSHTTPURLResponse.class] ? ((NSHTTPURLResponse*)r).statusCode : 0;
            dispatch_semaphore_signal(done);
        }];
        [task resume];
        while (dispatch_semaphore_wait(done, dispatch_time(DISPATCH_TIME_NOW, 100 * NSEC_PER_MSEC)) != 0) {
            if (cancel && cancel->load()) {
                [task cancel];
                dispatch_semaphore_wait(done, DISPATCH_TIME_FOREVER);
                if (error) *error = "cancelled";
                return false;
            }
        }
        if (err) { if (error) *error = err.localizedDescription.UTF8String; return false; }
        const std::string text = data ? std::string((const char*)data.bytes, data.length) : std::string();
        if (status >= 400) {
            // the provider explains in its JSON: {"error": {"message": ...}}; the text goes on to the caller as the answer
            if (text.empty()) { if (error) *error = "the provider answered " + std::to_string(status); return false; }
            *response = text;
            return true;
        }
        *response = text;
        return true;
    };
}
