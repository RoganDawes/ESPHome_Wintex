/* Copyright (C) 2020 Oxan van Leeuwen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "stream_server.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2021, 10, 0)
#include "esphome/components/network/util.h"
#endif


static const char *TAG = "streamserver";

using namespace esphome;

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "(%d) Setting up stream server ...", this->port_);
    this->recv_buf_.reserve(128);

    this->server_ = AsyncServer(this->port_);
    this->server_.begin();
    this->server_.onClient([this](void *h, AsyncClient *tcpClient) {
        if(tcpClient == nullptr)
            return;

        if(this->clients_.size() > 0) {
            ESP_LOGW(TAG, "(%d) Got a second connection, dropping it!", this->port_);
            tcpClient->close();
            return;
        }
        this->clients_.push_back(std::unique_ptr<Client>(new Client(tcpClient, this->recv_buf_)));
    }, this);
}

void StreamServerComponent::loop() {
    this->cleanup();
    this->read();
    this->write();
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](std::unique_ptr<Client> &client) { return !client->disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    for (auto it = last_client; it != this->clients_.end(); it++)
        ESP_LOGD(TAG, "(%d) Client %s disconnected", this->port_, (*it)->identifier.c_str());

    this->clients_.erase(last_client, this->clients_.end());
}

std::string StreamServerComponent::hexencode(const uint8_t *data, uint32_t off, uint32_t len) {
  char buf[20];
  std::string res;
  for (uint32_t i = off; i < off + 16; i++) {
    if (i + 1 <= off + len) {
      sprintf(buf, "%02X ", data[i]);
    } else {
      sprintf(buf, "   ");
    }
    res += buf;
  }
  res += "|";
  for (uint32_t i = off; i < off + 16; i++) {
    if (i + 1 <= off + len) {
      if (data[i]>0x1F && data[i]<0x7F) {
        sprintf(buf, "%c", data[i]);
      } else {
        sprintf(buf, ".");
      }
    } else {
      sprintf(buf, " ");
    }
    res += buf;
  }
  res += "|";
  return res;
}

void StreamServerComponent::read() {
    int len;
    while ((len = this->stream_->available()) > 0) {
        char buf[128];
        len = std::min(len, 128);
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2021, 10, 0)
        this->stream_->read_array(reinterpret_cast<uint8_t*>(buf), len);
#else
        this->stream_->readBytes(buf, len);
#endif
        for (int i = 0; i < len; i += 0x10) {
          int l = std::min(static_cast<int>(len)-i, 0x10);
          ESP_LOGD(TAG, "(%d) Read : %s", this->port_, this->hexencode((uint8_t *)buf, i, l).c_str());
        }
        for (auto const& client : this->clients_)
            client->tcp_client->write(buf, len);
    }
}

void StreamServerComponent::write() {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2021, 10, 0)
    this->stream_->write_array(this->recv_buf_);
    size_t len = this->recv_buf_.size();
    for (int i = 0; i < len; i += 0x10) {
        int l = std::min(static_cast<int>(len)-i, 0x10);
        ESP_LOGD(TAG, "(%d) Write: %s", this->port_, this->hexencode((uint8_t *)this->recv_buf_.data(), i, l).c_str());
    }
    this->recv_buf_.clear();
#else
    size_t len;
    while ((len = this->recv_buf_.size()) > 0) {
        this->stream_->write(this->recv_buf_.data(), len);
        for (int i = 0; i < len; i += 0x10) {
          int l = std::min(static_cast<int>(len)-i, 0x10);
          ESP_LOGD(TAG, "(%d) Write: %s", this->port_, this->hexencode((uint8_t *)this->recv_buf_.data(), i, l).c_str());
        }
        this->recv_buf_.erase(this->recv_buf_.begin(), this->recv_buf_.begin() + len);
    }
#endif
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u",
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2021, 10, 0)
                  esphome::network::get_ip_address().str().c_str(),
#else
                  network_get_address().c_str(),
#endif
                  this->port_);
}

void StreamServerComponent::on_shutdown() {
    for (auto &client : this->clients_)
        client->tcp_client->close(true);
}

StreamServerComponent::Client::Client(AsyncClient *client, std::vector<uint8_t> &recv_buf) :
        tcp_client{client}, identifier{client->remoteIP().toString().c_str()}, disconnected{false} {
    ESP_LOGD(TAG, "New client connected from %s", this->identifier.c_str());

    this->tcp_client->onError(     [this](void *h, AsyncClient *client, int8_t error)  { this->disconnected = true; });
    this->tcp_client->onDisconnect([this](void *h, AsyncClient *client)                { this->disconnected = true; });
    this->tcp_client->onTimeout(   [this](void *h, AsyncClient *client, uint32_t time) { this->disconnected = true; });

    this->tcp_client->onData([&](void *h, AsyncClient *client, void *data, size_t len) {
        if (len == 0 || data == nullptr)
            return;

        auto buf = static_cast<uint8_t *>(data);
        recv_buf.insert(recv_buf.end(), buf, buf + len);
    }, nullptr);
}

StreamServerComponent::Client::~Client() {
    delete this->tcp_client;
}

