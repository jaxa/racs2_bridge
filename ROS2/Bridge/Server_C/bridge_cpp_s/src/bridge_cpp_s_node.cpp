// SPDX-License-Identifier: Apache-2.0
#include <rclcpp/rclcpp.hpp>
#include <racs2_msg/msg/racs2_user_msg.hpp>
#include <libwebsockets.h>
#include "racs2_bridge_wire.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

class BridgeServer : public rclcpp::Node
{
    using Message = racs2_msg::msg::RACS2UserMsg;
    struct Frame
    {
        std::array<unsigned char, LWS_PRE + RACS2_MESSAGE_MAX> bytes{};
        size_t size{};
    };
public:
    BridgeServer() : Node("bridge_cpp_s")
    {
        host_ = declare_parameter<std::string>("wss_uri", "127.0.0.1");
        auto port = declare_parameter<int>("wss_port", 8765);
        if (port < 1 || port > 65535)
            throw std::invalid_argument("wss_port must be between 1 and 65535");
        lws_context_creation_info info{};
        info.iface = host_.c_str();
        info.port = port;
        info.protocols = protocols_;
        info.user = this;
        info.gid = -1;
        info.uid = -1;
        context_ = lws_create_context(&info);
        if (!context_)
            throw std::runtime_error("cannot create RACS2 WebSocket server");
        try
        {
            subscription_ = create_subscription<Message>("/RACS2Bridge", 10,
                [this](Message::ConstSharedPtr msg) { enqueue(*msg); });
            service_ = std::thread([this] {
                while (!stop_.load())
                    if (lws_service(context_, 0) < 0)
                        break;
            });
        }
        catch (...)
        {
            lws_context_destroy(context_);
            context_ = nullptr;
            throw;
        }
        RCLCPP_INFO(get_logger(), "RACS2 listening on ws://%s:%d", host_.c_str(), static_cast<int>(port));
    }
    ~BridgeServer() override
    {
        stop_ = true;
        lws_cancel_service(context_);
        if (service_.joinable())
            service_.join();
        lws_context_destroy(context_);
    }
private:
    void enqueue(const Message &msg)
    {
        if (msg.cfs_message_id > 0xffff || msg.body_data.size() > RACS2_BODY_MAX ||
            msg.body_data_length != msg.body_data.size())
        {
            RCLCPP_WARN(get_logger(), "Invalid RACS2 MID or body length");
            return;
        }
        if (!connected_.load())
            return;
        Frame frame;
        racs2_wire_set_mid(frame.bytes.data() + LWS_PRE,
                           static_cast<uint16_t>(msg.cfs_message_id));
        std::copy(msg.body_data.begin(), msg.body_data.end(),
                  frame.bytes.begin() + LWS_PRE + RACS2_HEADER_SIZE);
        frame.size = RACS2_HEADER_SIZE + msg.body_data.size();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!connected_.load() || pending_.size() >= 32)
                return;
            pending_.push_back(std::move(frame));
        }
        // Only the service thread calls lws_write / on_writable.
        lws_cancel_service(context_);
    }
    void publish_received()
    {
        if (!racs2_wire_valid_size(received_.size()))
            return;
        const auto end = std::find(received_.begin(),
                                   received_.begin() + RACS2_HEADER_SIZE, 0);
        std::string topic(received_.begin(), end);
        if (topic.empty())
            return;
        try
        {
            auto found = publishers_.find(topic);
            if (found == publishers_.end())
                found = publishers_.emplace(topic, create_publisher<Message>(topic, 10)).first;
            Message msg;
            msg.body_data.assign(received_.begin() + RACS2_HEADER_SIZE, received_.end());
            msg.body_data_length = static_cast<uint32_t>(msg.body_data.size());
            found->second->publish(msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Cannot publish RACS2 topic: %s", e.what());
        }
    }
    int handle(struct lws *wsi, enum lws_callback_reasons reason, void *in, size_t len)
    {
        switch (reason)
        {
            case LWS_CALLBACK_ESTABLISHED:
                if (peer_ && peer_ != wsi)
                    return -1;
                peer_ = wsi;
                received_.clear();
                connected_ = true;
                break;
            case LWS_CALLBACK_RECEIVE:
                if (lws_is_first_fragment(wsi))
                    received_.clear();
                if (!lws_frame_is_binary(wsi) || len > RACS2_MESSAGE_MAX - received_.size())
                    return -1;
                received_.insert(received_.end(), static_cast<uint8_t *>(in),
                                  static_cast<uint8_t *>(in) + len);
                if (lws_is_final_fragment(wsi) && lws_remaining_packet_payload(wsi) == 0)
                {
                    publish_received();
                    received_.clear();
                }
                break;
            case LWS_CALLBACK_EVENT_WAIT_CANCELLED:
                if (peer_)
                    lws_callback_on_writable(peer_);
                break;
            case LWS_CALLBACK_SERVER_WRITEABLE:
            {
                Frame frame;
                bool more;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (pending_.empty())
                        break;
                    frame = std::move(pending_.front());
                    pending_.pop_front();
                    more = !pending_.empty();
                }
                if (lws_write(wsi, frame.bytes.data() + LWS_PRE, frame.size,
                              LWS_WRITE_BINARY) < static_cast<int>(frame.size))
                    return -1;
                if (more)
                    lws_callback_on_writable(wsi);
                break;
            }
            case LWS_CALLBACK_CLOSED:
                if (peer_ == wsi)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    connected_ = false;
                    peer_ = nullptr;
                    pending_.clear();
                    received_.clear();
                }
                break;
            default:
                break;
        }
        return 0;
    }
    static int callback(struct lws *wsi, enum lws_callback_reasons reason,
                         void *, void *in, size_t len)
    {
        auto self = static_cast<BridgeServer *>(lws_context_user(lws_get_context(wsi)));
        if (!self)
            return 0;
        try { return self->handle(wsi, reason, in, len); }
        catch (...) { return -1; } // Never unwind a C callback.
    }
    inline static const lws_protocols protocols_[] = {
        {"example-protocol", callback, 0, RACS2_MESSAGE_MAX, 0, nullptr, 0},
        {nullptr, nullptr, 0, 0, 0, nullptr, 0}};
    std::string host_;
    lws_context *context_{};
    lws *peer_{};
    std::atomic<bool> stop_{false};
    std::atomic<bool> connected_{false};
    std::thread service_;
    std::mutex mutex_;
    std::deque<Frame> pending_;
    std::vector<uint8_t> received_;
    std::map<std::string, rclcpp::Publisher<Message>::SharedPtr> publishers_;
    rclcpp::Subscription<Message>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<BridgeServer>()); }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bridge_cpp_s"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
