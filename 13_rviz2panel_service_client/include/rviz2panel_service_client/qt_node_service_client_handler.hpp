#ifndef RVIZ2PANEL_SERVICE_CLIENT_HANDLER_HPP_
#define RVIZ2PANEL_SERVICE_CLIENT_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <memory>
#include "ros_types.hpp"

namespace rviz2panel_service_client
{

template<typename ServiceT>
class QtNodeServiceClientHandler
{
public:
  QtNodeServiceClientHandler() {}

  void setRosNodePtr(const rclcpp::Node::SharedPtr node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  void initializeClient(const std::string service_name)
  {
    client_ = node_ptr_->create_client<ServiceT>(service_name);
  }

  void finalizeClient()
  {
    client_.reset();
  }

  void sendRequest(const typename ServiceT::Request::SharedPtr request)
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Service %s is not available",
        client_->get_service_name());
      return;
    }

    auto future = client_->async_send_request(
      request,
      [this](typename rclcpp::Client<ServiceT>::SharedFuture response) {
        onResponseReceived(response);
      });
  }

  bool getResponse(typename ServiceT::Response::SharedPtr & response)
  {
    if (response_queue_.empty()) {
      return false;
    }

    response = response_queue_.front();
    response_queue_.pop();
    return true;
  }

  bool getLatestResponse(typename ServiceT::Response::SharedPtr & response)
  {
    if (response_queue_.empty()) {
      return false;
    }

    response = response_queue_.back();
    while (!response_queue_.empty()) {
      response_queue_.pop();
    }
    return true;
  }

private:
  void onResponseReceived(const typename rclcpp::Client<ServiceT>::SharedFuture response)
  {
    response_queue_.push(response.get());

    while (response_queue_.size() > max_queue_size_) {
      response_queue_.pop();
    }
  }

  rclcpp::Node::SharedPtr node_ptr_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  std::queue<typename ServiceT::Response::SharedPtr> response_queue_;
  static constexpr uint16_t max_queue_size_ = 10;
};

} // namespace rviz2panel_service_client

#endif // RVIZ2PANEL_SERVICE_CLIENT_HANDLER_HPP_
