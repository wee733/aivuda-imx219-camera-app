// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "cuda_buffer_backend/cuda_buffer_backend.hpp"
#include "cuda_buffer_backend_msgs/msg/cuda_buffer_descriptor.hpp"

namespace
{
using cuda_buffer_backend::CudaBufferBackend;
using cuda_buffer_backend::CudaBufferImpl;
using Descriptor = cuda_buffer_backend_msgs::msg::CudaBufferDescriptor;

void expect_empty_cpu(CudaBufferBackend & backend, const Descriptor * descriptor)
{
  auto erased = backend.from_descriptor_with_endpoint(descriptor, {});
  // Reproduce the Lyrical consumer, including its transfer of ownership.
  ASSERT_NE(erased.get(), nullptr);
  rosidl::Buffer<uint8_t> buffer(std::unique_ptr<rosidl::BufferImplBase<uint8_t>>{
      static_cast<rosidl::BufferImplBase<uint8_t> *>(erased.release())});
  EXPECT_EQ(buffer.get_backend_type(), "cpu");
  EXPECT_TRUE(buffer.empty());
}

TEST(CudaDescriptor, MalformedInputIsSafeWithoutACudaDevice)
{
  CudaBufferBackend backend;
  expect_empty_cpu(backend, nullptr);
  Descriptor d;
  expect_empty_cpu(backend, &d);
  d.element_type_name = typeid(uint8_t).name();
  d.use_ipc = true;
  d.vmm_pid = getpid();
  d.vmm_socket_path = "/unused";
  d.size = 2;
  d.vmm_block_size = 1;
  d.ipc_uid = 1;
  expect_empty_cpu(backend, &d);
  d.vmm_block_size = 2;
  d.ipc_uid = 0;
  expect_empty_cpu(backend, &d);
}

TEST(CudaDescriptor, InvalidDomainDeclinesIpcWithoutEscapingDiscovery)
{
  const char * old = std::getenv("ROS_DOMAIN_ID");
  const bool had_domain = old != nullptr;
  const std::string saved = old ? old : "";
  ASSERT_EQ(setenv("ROS_DOMAIN_ID", "12x", 1), 0);
  CudaBufferBackend backend;
  rmw_topic_endpoint_info_t endpoint{};
  endpoint.endpoint_type = RMW_ENDPOINT_SUBSCRIPTION;
  EXPECT_NO_THROW(backend.on_creating_endpoint(endpoint));
  EXPECT_FALSE(backend.on_discovering_endpoint(endpoint, {}, {{"cuda", ""}}).first);
  EXPECT_FALSE(backend.create_descriptor_with_endpoint(nullptr, endpoint));
  EXPECT_EQ(had_domain ? setenv("ROS_DOMAIN_ID", saved.c_str(), 1) : unsetenv("ROS_DOMAIN_ID"), 0);
}

TEST(CudaDescriptor, FailureAfterImportDoesNotExposeOrRetainTheBlock)
{
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0 ||
    !CudaBufferImpl<uint8_t>::is_pool_ipc_capable())
  {
    GTEST_SKIP() << "Requires a CUDA VMM IPC capable device";
  }
  CudaBufferBackend backend;
  rmw_topic_endpoint_info_t endpoint{};
  endpoint.endpoint_type = RMW_ENDPOINT_SUBSCRIPTION;
  const auto pid = getpid();
  std::memcpy(endpoint.endpoint_gid, &pid, sizeof(pid));
  backend.on_creating_endpoint(endpoint);

  CudaBufferImpl<uint8_t> source(32);
  auto desc = std::static_pointer_cast<Descriptor>(
    backend.create_descriptor_with_endpoint(&source, endpoint));
  ASSERT_TRUE(desc);
  EXPECT_TRUE(std::all_of(desc->ipc_event_handle.begin(), desc->ipc_event_handle.end(),
    [](uint8_t byte) {return byte == 0;}));
  auto pool = cuda_buffer_backend::get_or_create_global_pool();
  auto * block = pool->find_block_for_va(reinterpret_cast<CUdeviceptr>(
      source.get_cuda_buffer().get_device_ptr()));
  ASSERT_NE(block, nullptr);
  ASSERT_NE(block->ipc_meta, nullptr);
  const auto refs = block->ipc_meta->refcount.load();
  {
    auto ok = backend.from_descriptor_with_endpoint(desc.get(), endpoint);
    ASSERT_NE(ok.get(), nullptr);
    auto * impl = static_cast<rosidl::BufferImplBase<uint8_t> *>(ok.get());
    EXPECT_EQ(impl->get_backend_type(), "cuda");
    EXPECT_EQ(impl->size(), 32u);
    EXPECT_EQ(block->ipc_meta->refcount.load(), refs + 1);
  }
  EXPECT_EQ(block->ipc_meta->refcount.load(), refs);

  auto stale = *desc;
  stale.ipc_uid = desc->ipc_uid == 1 ? 2 : 1;
  expect_empty_cpu(backend, &stale);
  EXPECT_EQ(block->ipc_meta->refcount.load(), refs);

  const auto published = block->ipc_meta->publish_timestamp_us.exchange(0);
  expect_empty_cpu(backend, desc.get());
  block->ipc_meta->publish_timestamp_us.store(published);
  EXPECT_EQ(block->ipc_meta->refcount.load(), refs);

  desc->ipc_event_handle.fill(0xff);  // Failed event open after a successful block import.
  expect_empty_cpu(backend, desc.get());
  EXPECT_EQ(block->ipc_meta->refcount.load(), refs);
}
}  // namespace
