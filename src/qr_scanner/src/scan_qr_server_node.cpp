#include "qr_scanner/scan_qr_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<qr_scanner::ScanQrServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}













