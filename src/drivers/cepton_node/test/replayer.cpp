/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.

#include <rcutils/cmdline_parser.h>
#include <cepton_sdk.hpp>
#include <capture.hpp>
#include <apex_init/apex_init.hpp>
#include <logging/logging_macros.hpp>
#include <calibration_packet.hpp>
#include <string>
#include <thread>


int32_t main(const int32_t argc, char8_t * argv[])
{
  if (APEX_RET_OK != apex::pre_init(argc, argv)) {
    throw std::runtime_error("Could not pre-init apex");
  }
  const char8_t * ip = "127.0.0.1";
  const char8_t * arg = rcutils_cli_get_option(argv, &argv[argc], "--ip");
  if (nullptr != arg) {
    ip = arg;
  }
  uint16_t port = 8808U;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--port");
  if (nullptr != arg) {
    port = static_cast<uint16_t>(std::stoi(arg));
  }
  const char8_t * filename = "";
  arg = rcutils_cli_get_option(argv, &argv[argc], "--file");
  if (nullptr != arg) {
    filename = arg;
  }
  int32_t init_period_s = 3;
  arg = rcutils_cli_get_option(argv, &argv[argc], "--init_period");
  if (nullptr != arg) {
    init_period_s = std::atoi(arg);
  }

  apex::networking::udp::UdpVoidSender sender(ip, port);
  cepton_sdk::Options options{cepton_sdk::create_options()};
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  cepton_sdk::SensorError ret;
  ret = cepton_sdk::initialize(CEPTON_SDK_VERSION, options, NULL, NULL);
  if (ret) {throw ret;}
  cepton_sdk::Capture capture;
  ret = capture.open_for_read(filename);
  if (ret) {throw ret;}

  if (APEX_RET_OK != apex::post_init()) {
    throw std::runtime_error("Could not post-init apex");
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Send out special calibration data
  const auto start = std::chrono::steady_clock::now();
  const auto init_period = std::chrono::seconds{init_period_s};
  while (std::chrono::steady_clock::now() - start < init_period) {
    if (APEX_RET_OK != sender.send(calibration_pkt, sizeof(calibration_pkt))) {
      throw std::runtime_error("Failed to send initialization packet");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{7LL});
  }

  // Notify launchtest to start the listeners
  APEX_INFO_R("Spoofer(s) number is: ", 1u);
  std::cout << "Spoofer(s) number is: 1" << std::endl;

  // Send out normal pcap data
  while (rclcpp::ok()) {
    cepton_sdk::Capture::PacketHeader header;
    const uint8_t * buff;
    ret = capture.next_packet(header, buff);
    if (static_cast<int32_t>(CEPTON_ERROR_EOF) == static_cast<int32_t>(ret)) {
      capture.rewind();
    } else if (ret) {throw ret;}
    if (APEX_RET_OK != sender.send(buff, header.data_size)) {
      throw std::runtime_error("Failed to normal data");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1LL));
  }
  capture.close();

  (void)cepton_sdk_deinitialize();

  return 0;
}
