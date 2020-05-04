/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief main function for cepton_node

#include "cepton_node/cepton_block_node.hpp"
#include <apex_init/apex_init.hpp>
#include <memory>
#include <string>

int32_t main(const int32_t argc, char8_t * argv[])
{
  int32_t ret = 0;
  try {
    if (apex::pre_init(argc, argv) != APEX_RET_OK) {
      throw std::runtime_error("Can't pre-init Apex");
    }
    using apex_auto::drivers::cepton_node::CeptonBlockNode;
    const std::shared_ptr<CeptonBlockNode> nd_ptr = std::make_shared<CeptonBlockNode>(
      "cepton_block_node",
      std::chrono::milliseconds(11LL));  // semi-hack: We know the packet rate for this component
    if (apex::post_init() != APEX_RET_OK) {
      throw std::runtime_error("Can't post-init Apex");
    }
    nd_ptr->run();

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100LL));
    }

    nd_ptr->stop();
    nd_ptr->join();
  } catch (const std::exception & e) {
    std::cerr << e.what();
    APEX_ERROR_R(e.what());
    ret = __LINE__;
  } catch (...) {
    std::cerr << "Unknown error occured";
    APEX_FATAL_R("Unknown error occured");
    ret = __LINE__;
  }

  return ret;
}
