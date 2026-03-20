#include "zmq_bridge.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

namespace slam {

ZMQSubscriber::ZMQSubscriber(const std::string& address)
    : context_(1), socket_(context_, zmq::socket_type::sub), address_(address) 
    {
    socket_.set(zmq::sockopt::conflate, 1);
    
    socket_.connect(address_);
    std::cout << "[ZMQSubscriber] Connected to " << address_ << std::endl;
}

ZMQSubscriber::~ZMQSubscriber() 
{
    socket_.close();
    std::cout << "[ZMQSubscriber] Disconnected" << std::endl;
}

void ZMQSubscriber::subscribe(const std::string& topic) 
{
    socket_.set(zmq::sockopt::subscribe, topic);
    std::cout << "[ZMQSubscriber] Subscribed to topic: " << topic << std::endl;
}


bool ZMQSubscriber::receiveMessage(
    std::function<void(const std::string&, const std::string&)> callback,
    int timeout_ms) {
    
    // Set receive timeout
    if (timeout_ms >= 0) {
        socket_.set(zmq::sockopt::rcvtimeo, timeout_ms);
    }
    
    zmq::message_t message;
    zmq::recv_result_t result = socket_.recv(message, zmq::recv_flags::none);
    
    if (!result) {
        return false; // Timeout or error
    }
    
    // Convert message to string
    std::string full_message(static_cast<char*>(message.data()), message.size());
    
    // Split into topic and message (format: "topic message_json")
    size_t space_pos = full_message.find(' ');
    if (space_pos == std::string::npos) {
        std::cerr << "[ZMQSubscriber] Invalid message format: " << full_message << std::endl;
        return false;
    }
    
    std::string topic = full_message.substr(0, space_pos);
    std::string msg_content = full_message.substr(space_pos + 1);
    

    callback(topic, msg_content);
    
    return true;
}

void ZMQSubscriber::startListening(
    std::function<void(const std::string&, const std::string&)> callback) {
    
    std::cout << "[ZMQSubscriber] Starting listening loop..." << std::endl;
    
    while (true) {
        try {
            receiveMessage(callback, -1); // Blocking receive
        } catch (const zmq::error_t& e) {
            if (e.num() == EINTR) {
                std::cout << "[ZMQSubscriber] Interrupted" << std::endl;
                break;
            }
            std::cerr << "[ZMQSubscriber] Error: " << e.what() << std::endl;
        }
    }
}


ZMQPublisher::ZMQPublisher(const std::string& address)
    : context_(1), socket_(context_, zmq::socket_type::pub), address_(address) 
    {
    
    socket_.bind(address_);
    std::cout << "[ZMQPublisher] Bound to " << address_ << std::endl;
    
    // Small delay to allow subscribers to connect
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

ZMQPublisher::~ZMQPublisher() 
{
    socket_.close();
    std::cout << "[ZMQPublisher] Closed" << std::endl;
}

void ZMQPublisher::publishMessage(const std::string& topic, const std::string& message) 
{
    std::string full_message = topic + " " + message;
    socket_.send(zmq::buffer(full_message), zmq::send_flags::none);
}

} // namespace slam
