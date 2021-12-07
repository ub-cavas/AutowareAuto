Fake Test Node {#fake-test-node-design}
=====================

# What this package provides

When writing an integration test for a node in C++ using GTest, there is quite some boilerplate code
that needs to be written to set up a fake node that would publish expected messages on an expected
topic and subscribes to messages on some other topic. This is usually implemented as a custom GTest
fixture.

This package contains a library that introduces two utility classes that can be used in place of
custom fixtures described above to write integration tests for a node:
- `autoware::tools::testing::FakeTestNode` - to use as a custom test fixture with `TEST_F` tests
- `autoware::tools::testing::FakeTestNodeParametrized` - to use a custom test fixture with the
  parametrized `TEST_P` tests (accepts a template parameter that gets forwarded to
  `testing::TestWithParam<T>`)

These fixtures take care of initializing and re-initializing rclcpp as well as of checking that all
subscribers and publishers have a match, thus reducing the amount of boilerplate code that the user
needs to write.

# How to use this library
After including the relevant header the user can use a typedef to use a custom fixture name and use
the provided classes as fixtures in `TEST_F` and `TEST_P` tests directly.

## Example usage
Let's say there is a node `NodeUnderTest` that requires testing. It just
subscribes to `std_msgs::msg::Int32` messages and publishes a
`std_msgs::msg::Bool` to indicate that the input is positive. To test such a
node the following code can be used utilizing the
`autoware::tools::testing::FakeTestNode`:

```cpp
using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

/// @test Test that we can use a non-parametrized test.
TEST_F(FakeNodeFixture, Test) {
  Int32 msg{};
  msg.data = 15;
  const auto node = std::make_shared<NodeUnderTest>();

  Bool::SharedPtr last_received_msg{};
  auto fake_odom_publisher = create_publisher<Int32>("/input_topic");
  auto result_odom_subscription = create_subscription<Bool>("/output_topic", *node,
    [&last_received_msg](const Bool::SharedPtr msg) {last_received_msg = msg;});

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_odom_publisher->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }
  EXPECT_TRUE(last_received_msg->data);
  SUCCEED();
}
```

Here only the `TEST_F` example is shown but a `TEST_P` usage is very similar with a little bit more
boilerplate to set up all the parameter values, see `test_fake_test_node.cpp` for an example usage.

## Using namespaces

When a test relies on ROS communication,
it becomes susceptibles to interferences comming from nodes outside of the tests
(e.g., from other tests running in parallel).
ROS namespaces can be used to isolate a test and ensure it will not be bothered by outside messages.
In order to set the namespace for the `FakeTestNode`,
it should be extended into a new class which sets the namespace in its constructor
via the `set_namespace()` function. This new class can then be used as test fixture instead of the `FakeTestNode`.
```C++
class FakeNodeFixtureWithNamespace : public FakeTestNode
{
public:
  FakeNodeFixtureWithNamespace() {set_namespace("/namespace");}
};
```

The namespace also applies to the `tf` and `tf_static` topics.
If you are testing a node that uses these topics (e.g., when the node uses a `TransformListener`),
then it is important to use the following `NodeOptions` for the node.
```C++
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args",
      // Set node namespace
      "-r", "__ns:=/namespace"
      // Remap tf and tf_static so that they use the node namespace
      "-r", "/tf:=tf",
      "-r", "/tf_static:=tf_static"});
```
