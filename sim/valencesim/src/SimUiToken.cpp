// SimUiToken -- implementation. See SimUiToken.h; nothing is restated here.

#include "SimUiToken.h"

#include <cstdio>

#include <ixwebsocket/IXHttpServer.h>

#include "valence/core/crypto.hpp"

namespace valence {

namespace {

uint32_t nowMs() { return uint32_t(deviceNowUs() / 1000); }

}  // namespace

SimUiToken::SimUiToken() = default;

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
    std::array<std::byte, kTokenBytes> tok{};
    {
        std::lock_guard<std::mutex> lk(_m);
        if (_everMinted && (now - _lastMintMs) < kMinIntervalMs) {
            body = "{\"ok\":false,\"error\":\"rate_limited\"}";
            return 429;
        }
        _everMinted = true;
        _lastMintMs = now;
        for (auto& b : tok) b = std::byte(_rd() & 0xFF);

        // Oldest-expiring slot loses, used slots first, exactly as on the P4.
        size_t victim = 0;
        for (size_t i = 1; i < kSlots; ++i) {
            if (_slots[i].used && !_slots[victim].used) { victim = i; continue; }
            if (_slots[i].used == _slots[victim].used && _slots[i].expiresMs < _slots[victim].expiresMs)
                victim = i;
        }
        _slots[victim].token = tok;
        _slots[victim].expiresMs = now + kTtlMs;
        _slots[victim].used = false;
    }

    std::array<char, kTokenBytes * 2 + 1> hex{};
    static constexpr char kHex[] = "0123456789abcdef";
    for (size_t i = 0; i < kTokenBytes; ++i) {
        hex[i * 2] = kHex[(uint8_t(tok[i]) >> 4) & 0x0F];
        hex[i * 2 + 1] = kHex[uint8_t(tok[i]) & 0x0F];
    }
    std::array<char, 128> out{};
    std::snprintf(out.data(), out.size(),
                  "{\"ok\":true,\"token\":\"%s\",\"ttl_ms\":%lu,\"tier\":\"control\"}", hex.data(),
                  static_cast<unsigned long>(kTtlMs));
    body = out.data();
    return 200;
}

bool SimUiToken::consume(std::span<const std::byte> token) {
    if (token.size() != kTokenBytes) return false;
    const uint32_t now = nowMs();
    SoftwareCrypto cmp;
    std::lock_guard<std::mutex> lk(_m);
    for (auto& s : _slots) {
        if (s.used) continue;
        if (int32_t(now - s.expiresMs) >= 0) {
            s.used = true;
            continue;
        }
        if (cmp.constantTimeEqual(std::span<const std::byte>(s.token), token)) {
            s.used = true;
            return true;
        }
    }
    return false;
}

}  // namespace valence
