// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cuda_buffer_backend/cuda_buffer_backend.hpp"

#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <typeinfo>

#include "cuda_buffer/cuda_buffer_ipc_manager.hpp"
#include "cuda_buffer/cuda_error.hpp"
#include "cuda_buffer/cuda_memory_pool.hpp"
#include "cuda_buffer_backend_msgs/msg/cuda_buffer_descriptor.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcutils/logging_macros.h"
#include "rosidl_buffer/cpu_buffer_impl.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"


namespace cuda_buffer_backend
{

namespace
{
// Lyrical's endpoint callback carries a GID and kind, not an rcl context or DDS domain.
// This application launches one domain per process and exports it as canonical decimal.
size_t registry_domain()
{
  const char * value = std::getenv("ROS_DOMAIN_ID");
  if (!value || !*value) {
    return 0;
  }
  std::string_view text(value);
  unsigned int domain = 0;
  const char * end = text.data() + text.size();
  auto result = std::from_chars(text.data(), end, domain);
  if (result.ec != std::errc{} || result.ptr != end || domain > 232) {
    throw std::invalid_argument("ROS_DOMAIN_ID must be a decimal DDS domain in [0, 232]");
  }
  return domain;
}

rmw_gid_t endpoint_gid(const rmw_topic_endpoint_info_t & endpoint)
{
  rmw_gid_t gid{};
  gid.implementation_identifier = "";
  std::memcpy(gid.data, endpoint.endpoint_gid, sizeof(gid.data));
  return gid;
}

bool can_share(
  const host_endpoint_manager::LocalityInfo & peer,
  const host_endpoint_manager::HostEndpointManager & manager)
{
  using host_endpoint_manager::EndpointLocality;
  return peer.found && peer.ipc_capable &&
         (peer.locality == EndpointLocality::INTRA_PROCESS ||
         (peer.locality == EndpointLocality::INTER_PROCESS_SAME_HOST &&
         peer.device_id == manager.get_device_id() && peer.uid == manager.get_uid()));
}
}  // namespace

CudaBufferBackend::CudaBufferBackend()
{
}

const rosidl_message_type_support_t *
CudaBufferBackend::get_descriptor_type_support() const
{
  return rosidl_typesupport_cpp::get_message_type_support_handle<
    cuda_buffer_backend_msgs::msg::CudaBufferDescriptor>();
}

std::shared_ptr<void>
CudaBufferBackend::create_empty_descriptor() const
{
  return std::make_shared<cuda_buffer_backend_msgs::msg::CudaBufferDescriptor>();
}

std::shared_ptr<host_endpoint_manager::HostEndpointManager>
CudaBufferBackend::get_endpoint_manager() const
{
  std::lock_guard<std::mutex> lock(manager_mutex_);

  if (!endpoint_manager_) {
    try {
      const auto domain = registry_domain();
      if (!CudaBufferImpl<uint8_t>::is_pool_ipc_capable()) {
        return nullptr;
      }
      endpoint_manager_ = host_endpoint_manager::HostEndpointManager::get_instance(domain);
      endpoint_manager_->set_ipc_capable(true);
    } catch (const std::exception & error) {
      RCUTILS_LOG_WARN_ONCE_NAMED("cuda_buffer_backend",
        "CUDA IPC registration unavailable: %s", error.what());
      return nullptr;  // Never register in domain zero after a configuration error.
    }
  }

  return endpoint_manager_;
}

void CudaBufferBackend::on_creating_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  auto manager = get_endpoint_manager();
  if (!manager) {
    return;
  }

  const auto gid = endpoint_gid(endpoint_info);

  if (endpoint_info.endpoint_type == RMW_ENDPOINT_PUBLISHER) {
    manager->register_publisher(gid, endpoint_info.topic_type);
  } else if (endpoint_info.endpoint_type == RMW_ENDPOINT_SUBSCRIPTION) {
    manager->register_subscription(gid, endpoint_info.topic_type);
  }
}

std::pair<bool, std::vector<std::set<uint32_t>>>
CudaBufferBackend::on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  (void)existing_endpoints;

  const host_endpoint_manager::GidKey key(endpoint_info.endpoint_gid);
  const bool supports_cuda = endpoint_supported_backends.find("cuda") !=
    endpoint_supported_backends.end();
  auto manager = supports_cuda ? get_endpoint_manager() : nullptr;
  if (!manager) {
    std::lock_guard<std::mutex> lock(ipc_cache_mutex_);
    rejected_peers_.insert(key);
    return {false, {}};
  }

  const auto gid = endpoint_gid(endpoint_info);

  manager->refresh_from_remote();

  auto locality_info = manager->query_endpoint_locality(gid);

  const bool is_ipc_capable = can_share(locality_info, *manager);

  if (locality_info.found) {
    std::lock_guard<std::mutex> lock(ipc_cache_mutex_);
    if (is_ipc_capable) {
      rejected_peers_.erase(key);
    } else {
      rejected_peers_.insert(key);
    }
  }

  return {is_ipc_capable, {}};
}

std::shared_ptr<void> CudaBufferBackend::create_descriptor_with_endpoint(
  const void * impl,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  host_endpoint_manager::GidKey key(endpoint_info.endpoint_gid);

  {
    std::lock_guard<std::mutex> lock(ipc_cache_mutex_);
    if (rejected_peers_.count(key)) {
      return nullptr;
    }
  }

  const auto gid = endpoint_gid(endpoint_info);
  auto manager = get_endpoint_manager();
  if (!manager) {
    return nullptr;
  }
  auto locality = manager->query_endpoint_locality(gid);
  if (!locality.found) {
    // DDS discovery can precede remote HEM registration. Retry in our domain,
    // but never export a descriptor for an unconfirmed peer.
    manager->refresh_from_remote();
    locality = manager->query_endpoint_locality(gid);
  }
  if (!can_share(locality, *manager)) {
    return nullptr;
  }

  auto * cuda_impl = dynamic_cast<CudaBufferImpl<uint8_t> *>(
    const_cast<rosidl::BufferImplBase<uint8_t> *>(
      static_cast<const rosidl::BufferImplBase<uint8_t> *>(impl)));
  if (!cuda_impl) {
    return nullptr;
  }

  // The camera and plugin DSOs must use libcuda_buffer's out-of-line pool.
  // Plain 0.1.2 headers provide only a per-DSO inline pool; reject that build.
  auto pool = cuda_buffer_backend::get_or_create_global_pool();
  if (!pool || !pool->is_ipc_capable()) {
    return nullptr;
  }

  CUdeviceptr dev_ptr = reinterpret_cast<CUdeviceptr>(
    cuda_impl->get_cuda_buffer().get_device_ptr());

  VmmBlock * block = pool->find_block_for_va(dev_ptr);
  if (!block || block->exported_fd < 0 || !block->ipc_meta || cuda_impl->size() > block->size) {
    return nullptr;
  }

  cuda_impl->get_cuda_buffer().finalize_write_handle();

  cudaEvent_t write_event = cuda_impl->get_cuda_buffer().get_write_event();

  if (!write_event) {
    // No event to order either an IPC reader or the serializer's CPU copy.
    CUDA_CHECK(cudaDeviceSynchronize());
    return nullptr;
  }
  cudaIpcEventHandle_t event_handle{};
  if (locality.locality == host_endpoint_manager::EndpointLocality::INTRA_PROCESS) {
    RCUTILS_LOG_WARN_ONCE_NAMED("cuda_buffer_backend",
      "Same-process CUDA IPC requires cudaEventSynchronize on the publish path. "
      "Enable intra-process communication to avoid this overhead.");
    // rmw_fastrtps catches serialization errors and skips this endpoint. Do not
    // advertise readiness if synchronization failed, or export a self IPC event.
    CUDA_CHECK(cudaEventSynchronize(write_event));
  } else if (cudaIpcGetEventHandle(&event_handle, write_event) != cudaSuccess) {
    (void)cudaGetLastError();
    return nullptr;  // The CPU serializer can still wait on the local event.
  }

  std::string socket_path = pool->register_block_for_ipc(block);
  if (socket_path.empty()) {
    return nullptr;
  }

  auto descriptor = std::make_shared<cuda_buffer_backend_msgs::msg::CudaBufferDescriptor>();
  descriptor->size = cuda_impl->size();
  descriptor->element_type_name = typeid(uint8_t).name();
  descriptor->device_id = cuda_impl->get_device_id();
  descriptor->use_ipc = true;
  descriptor->vmm_pid = static_cast<int32_t>(getpid());
  descriptor->vmm_block_id = block->block_id;
  descriptor->vmm_block_size = block->size;
  descriptor->vmm_socket_path = socket_path;
  descriptor->ipc_uid = pool->assign_uid(block);
  static_assert(sizeof(event_handle) == sizeof(descriptor->ipc_event_handle));
  std::memcpy(descriptor->ipc_event_handle.data(), &event_handle, sizeof(event_handle));

  return descriptor;
}

std::unique_ptr<void, void (*)(void *)> CudaBufferBackend::from_descriptor_with_endpoint(
  const void * descriptor_ptr,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;

  const auto * descriptor =
    static_cast<const cuda_buffer_backend_msgs::msg::CudaBufferDescriptor *>(descriptor_ptr);

  auto delete_impl = [](void * p) {delete static_cast<rosidl::BufferImplBase<uint8_t> *>(p);};
  try {
    if (!descriptor || descriptor->element_type_name != typeid(uint8_t).name() ||
      !descriptor->use_ipc || descriptor->vmm_pid <= 0 || descriptor->vmm_socket_path.empty() ||
      descriptor->ipc_uid == 0 || descriptor->size == 0 ||
      descriptor->size > descriptor->vmm_block_size)
    {
      throw CudaError("Invalid CUDA IPC descriptor");
    }
    const bool has_event = std::any_of(
      descriptor->ipc_event_handle.begin(), descriptor->ipc_event_handle.end(),
      [](uint8_t byte) {return byte != 0;});
    if (!has_event && descriptor->vmm_pid != getpid()) {
      throw CudaError("Missing inter-process CUDA write event");
    }
    int local_device_id = 0;
    CUDA_CHECK(cudaGetDevice(&local_device_id));
    if (descriptor->device_id != local_device_id) {
      throw CudaError("Cross-device IPC not implemented");
    }
    auto imported = CudaVmmIPCManager::import_block(
      descriptor->vmm_socket_path, descriptor->vmm_pid, descriptor->vmm_block_id,
      descriptor->vmm_block_size, descriptor->ipc_uid);
    IPCMetadata * meta = imported.ipc_meta;
    if (!meta) {
      // import_block skips its UID check and refcount when metadata is missing.
      throw CudaError("Cannot verify IPC block ownership without shared metadata");
    }
    CudaBuffer imported_buffer(
      reinterpret_cast<void *>(imported.va), descriptor->size, descriptor->device_id,
      [meta](uint8_t *) {meta->refcount.fetch_sub(1, std::memory_order_release);});
    if (meta->publish_timestamp_us.load(std::memory_order_acquire) == 0 ||
      meta->uid.load(std::memory_order_acquire) != descriptor->ipc_uid)
    {
      throw CudaError("Recycled CUDA IPC block");
    }
    if (has_event) {
      cudaIpcEventHandle_t event_handle;
      std::memcpy(&event_handle, descriptor->ipc_event_handle.data(), sizeof(event_handle));
      cudaEvent_t event = nullptr;
      const auto error = cudaIpcOpenEventHandle(&event, event_handle);
      if (error != cudaSuccess) {
        (void)cudaGetLastError();
        throw CudaError(__FILE__, __LINE__, "cudaIpcOpenEventHandle", error);
      }
      imported_buffer.set_write_event(event, true);
    }
    return {new CudaBufferImpl<uint8_t>(std::move(imported_buffer), descriptor->size), delete_impl};
  } catch (const std::exception & error) {
    // Lyrical Fast DDS unconditionally constructs a Buffer from our result and
    // does not catch import exceptions. The descriptor contains no CPU pixels.
    RCUTILS_LOG_WARN_NAMED("cuda_buffer_backend",
      "Unusable CUDA descriptor (%s); returning an empty CPU buffer", error.what());
    return {new rosidl::CpuBufferImpl<uint8_t>(), delete_impl};
  }
}
}  // namespace cuda_buffer_backend

PLUGINLIB_EXPORT_CLASS(
  cuda_buffer_backend::CudaBufferBackend,
  rosidl::BufferBackend)
