// SimUiToken -- implementation. See SimUiToken.h; nothing is restated here.

#include "SimUiToken.h"

#include <array>
#include <cstdio>
#include <optional>
#include <random>

#include <ixwebsocket/IXHttpServer.h>

namespace valence {

namespace {

uint32_t nowMs() { return uint32_t(deviceNowUs() / 1000); }

}  // namespace

SimUiToken::SimUiToken() {
    std::random_device rd;
    for (auto& b : _secret) b = std::byte(rd() & 0xFF);
}

SimUiToken::~SimUiToken() { stop(); }

bool SimUiToken::begin(uint16_t port, std::string& err) {
    _server = std::make_unique<ix::HttpServer>(port, "127.0.0.1");
    _server->setOnConnectionCallback(
        [this](ix::HttpRequestPtr req, std::shared_ptr<ix::ConnectionState>) -> ix::HttpResponsePtr {
            // ---- DO NOT ADD CORS HEADERS: their ABSENCE is the mechanism ----
            ix::WebSocketHttpHeaders h;
            h["Connection"] = "close";
            if (req->method != "GET" || req->uri != "/uitoken") {
                return std::make_shared<ix::HttpResponse>(404, "Not Found", ix::HttpErrorCode::Ok,
                                                          h, std::string("not found"));
            }
            std::string body;
            const int code = mint(body);
            h["Content-Type"] = "application/json";
            if (code == 200) h["Cache-Control"] = "no-store";
            return std::make_shared<ix::HttpResponse>(
                code, code == 200 ? "OK" : "Too Many Requests", ix::HttpErrorCode::Ok, h, body);
        });
    const auto res = _server->listen();
    if (!res.first) {
        err = res.second;
        _server.reset();
        return false;
    }
    _server->start();
    return true;
}

void SimUiToken::stop() {
    if (_server) {
        _server->stop();
        _server.reset();
    }
}

int SimUiToken::mint(std::string& body) {
    const uint32_t now = nowMs();
    std::optional<uint32_t> counter;
    {
        std::lock_guard<std::mutex> lk(_m);
        counter = _table.claimMint(now);
    }
    if (!counter) {
        body = "{\"ok\":false,\"error\":\"rate_limited\"}";
        return 429;
    }
    const UiTokenTable::Token tok = UiTokenTable::derive(_secret, *counter, now);
    {
        std::lock_guard<std::mutex> lk(_m);
        _table.install(tok, now);
    }

    std::array<char, UiTokenTable::kTokenBytes * 2 + 1> hex{};
    static constexpr char kHex[] = "0123456789abcdef";
    for (size_t i = 0; i < UiTokenTable::kTokenBytes; ++i) {
        hex[i * 2] = kHex[(uint8_t(tok[i]) >> 4) & 0x0F];
        hex[i * 2 + 1] = kHex[uint8_t(tok[i]) & 0x0F];
    }
    std::array<char, 128> out{};
    std::snprintf(out.data(), out.size(),
                  "{\"ok\":true,\"token\":\"%s\",\"ttl_ms\":%lu,\"tier\":\"control\"}", hex.data(),
                  static_cast<unsigned long>(UiTokenTable::kTtlMs));
    body = out.data();
    return 200;
}

bool SimUiToken::consume(std::span<const std::byte> token) {
    const uint32_t now = nowMs();
    std::lock_guard<std::mutex> lk(_m);
    return _table.consume(token, now, _cmp);
}

}  // namespace valence
