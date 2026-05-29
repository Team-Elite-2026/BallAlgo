#include "camera/CameraCapture.hpp"

#include "config.hpp"

#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <opencv2/imgproc.hpp>

#include <condition_variable>
#include <memory>
#include <sys/mman.h>
#include <unistd.h>

namespace ballalgo {

namespace {

struct PlaneMapping {
  void* base = nullptr;
  size_t mappedLength = 0;
  uint8_t* data = nullptr;

  ~PlaneMapping() {
    if (base) munmap(base, mappedLength);
  }

  PlaneMapping() = default;
  PlaneMapping(const PlaneMapping&) = delete;
  PlaneMapping& operator=(const PlaneMapping&) = delete;

  PlaneMapping(PlaneMapping&& other) noexcept
      : base(std::exchange(other.base, nullptr)),
        mappedLength(std::exchange(other.mappedLength, 0)),
        data(std::exchange(other.data, nullptr)) {}

  PlaneMapping& operator=(PlaneMapping&& other) noexcept {
    if (this == &other) return *this;
    if (base) munmap(base, mappedLength);
    base = std::exchange(other.base, nullptr);
    mappedLength = std::exchange(other.mappedLength, 0);
    data = std::exchange(other.data, nullptr);
    return *this;
  }
};

constexpr libcamera::PixelFormat kPreferredFormats[] = {
    libcamera::formats::BGR888,
    libcamera::formats::RGB888,
};

constexpr unsigned int kRequestBufferCount = 4;

PlaneMapping mapPlane(const libcamera::FrameBuffer::Plane& plane) {
  PlaneMapping mapped;
  const int fd = plane.fd.get();
  if (fd < 0 || plane.length == 0) return mapped;

  const long pageSize = sysconf(_SC_PAGESIZE);
  if (pageSize <= 0) return mapped;

  const size_t pageMask = static_cast<size_t>(pageSize - 1);
  const size_t alignedOffset = plane.offset & ~pageMask;
  const size_t delta = plane.offset - alignedOffset;
  const size_t mapLength = delta + plane.length;

  void* base = mmap(nullptr, mapLength, PROT_READ, MAP_SHARED, fd, alignedOffset);
  if (base == MAP_FAILED) return mapped;

  mapped.base = base;
  mapped.mappedLength = mapLength;
  mapped.data = static_cast<uint8_t*>(base) + delta;
  return mapped;
}

}  // namespace

struct CameraCapture::Impl {
  bool open();
  bool grab(cv::Mat& bgrOut);
  void close();

 private:
  bool configureStream();
  bool allocateRequests();
  bool queueRequest(libcamera::Request* request);
  bool copyRequestToMat(libcamera::Request* request, cv::Mat& bgrOut);
  void applyControls(libcamera::Request* request);
  void onRequestCompleted(libcamera::Request* request);

  std::unique_ptr<libcamera::CameraManager> manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  libcamera::Stream* stream_ = nullptr;
  libcamera::PixelFormat pixelFormat_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  unsigned int stride_ = 0;
  bool running_ = false;

  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::unordered_map<const libcamera::FrameBuffer*, std::vector<PlaneMapping>> bufferMappings_;
  std::mutex mutex_;
  std::condition_variable completedCv_;
  std::deque<libcamera::Request*> completedRequests_;
};

bool CameraCapture::Impl::open() {
  close();

  manager_ = std::make_unique<libcamera::CameraManager>();
  if (manager_->start() < 0) {
    std::fprintf(stderr, "ballalgo libcamera failed to start camera manager\n");
    close();
    return false;
  }

  const auto& cameras = manager_->cameras();
  if (cameras.empty()) {
    std::fprintf(stderr, "ballalgo libcamera found no cameras\n");
    close();
    return false;
  }

  camera_ = manager_->get(cameras.front()->id());
  if (!camera_) {
    std::fprintf(stderr, "ballalgo libcamera failed to get first camera\n");
    close();
    return false;
  }

  if (camera_->acquire() < 0) {
    std::fprintf(stderr, "ballalgo libcamera failed to acquire camera %s\n",
                 camera_->id().c_str());
    close();
    return false;
  }

  if (!configureStream() || !allocateRequests()) {
    close();
    return false;
  }

  camera_->requestCompleted.connect(this, &CameraCapture::Impl::onRequestCompleted);
  if (camera_->start() < 0) {
    std::fprintf(stderr, "ballalgo libcamera failed to start camera\n");
    close();
    return false;
  }

  running_ = true;
  for (auto& request : requests_) {
    if (camera_->queueRequest(request.get()) < 0) {
      std::fprintf(stderr, "ballalgo libcamera failed to queue initial requests\n");
      close();
      return false;
    }
  }

  return true;
}

bool CameraCapture::Impl::configureStream() {
  config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
  if (!config_ || config_->empty()) {
    std::fprintf(stderr, "ballalgo libcamera failed to generate viewfinder config\n");
    return false;
  }

  auto& streamConfig = config_->at(0);
  bool negotiatedPreferredFormat = false;

  for (const auto& preferredFormat : kPreferredFormats) {
    streamConfig.size = libcamera::Size(config::kFrameWidth, config::kFrameHeight);
    streamConfig.pixelFormat = preferredFormat;
    streamConfig.bufferCount = kRequestBufferCount;

    const auto status = config_->validate();
    if (status == libcamera::CameraConfiguration::Invalid) continue;
    if (streamConfig.pixelFormat == preferredFormat) {
      negotiatedPreferredFormat = true;
      break;
    }
  }

  if (!negotiatedPreferredFormat) {
    std::fprintf(stderr,
                 "ballalgo libcamera couldn't negotiate BGR/RGB output, got %s instead\n",
                 streamConfig.pixelFormat.toString().c_str());
    return false;
  }

  if (camera_->configure(config_.get()) < 0) {
    std::fprintf(stderr, "ballalgo libcamera camera configure failed\n");
    return false;
  }

  stream_ = streamConfig.stream();
  pixelFormat_ = streamConfig.pixelFormat;
  width_ = streamConfig.size.width;
  height_ = streamConfig.size.height;
  stride_ = streamConfig.stride;

  std::fprintf(stderr, "ballalgo libcamera configured %ux%u %s stride=%u buffers=%u\n",
               width_, height_, pixelFormat_.toString().c_str(), stride_,
               streamConfig.bufferCount);
  return true;
}

bool CameraCapture::Impl::allocateRequests() {
  allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
  const int allocated = allocator_->allocate(stream_);
  if (allocated < 0) {
    std::fprintf(stderr, "ballalgo libcamera buffer allocation failed: %d\n", allocated);
    return false;
  }

  const auto& buffers = allocator_->buffers(stream_);
  if (buffers.empty()) {
    std::fprintf(stderr, "ballalgo libcamera allocated zero buffers\n");
    return false;
  }

  for (size_t i = 0; i < buffers.size(); ++i) {
    libcamera::FrameBuffer* buffer = buffers[i].get();
    auto& mappedPlanes = bufferMappings_[buffer];
    mappedPlanes.reserve(buffer->planes().size());
    for (const auto& plane : buffer->planes()) {
      mappedPlanes.push_back(mapPlane(plane));
      if (!mappedPlanes.back().data) {
        std::fprintf(stderr, "ballalgo libcamera mmap failed for buffer %zu\n", i);
        return false;
      }
    }

    auto request = camera_->createRequest(i);
    if (!request) {
      std::fprintf(stderr, "ballalgo libcamera failed to create request %zu\n", i);
      return false;
    }

    if (request->addBuffer(stream_, buffer) < 0) {
      std::fprintf(stderr, "ballalgo libcamera failed to attach buffer %zu\n", i);
      return false;
    }

    applyControls(request.get());
    requests_.push_back(std::move(request));
  }

  return true;
}

void CameraCapture::Impl::applyControls(libcamera::Request* request) {
  auto& supported = camera_->controls();
  auto& controls = request->controls();

  if (supported.count(libcamera::controls::AeEnable.id())) {
    controls.set(libcamera::controls::AeEnable, false);
  }
  if (supported.count(libcamera::controls::AwbEnable.id())) {
    controls.set(libcamera::controls::AwbEnable, false);
  }
  if (config::kExposureUs > 0 &&
      supported.count(libcamera::controls::ExposureTime.id())) {
    controls.set(libcamera::controls::ExposureTime,
                 static_cast<int64_t>(config::kExposureUs));
  }
  if (config::kCameraGain > 0 &&
      supported.count(libcamera::controls::AnalogueGain.id())) {
    controls.set(libcamera::controls::AnalogueGain,
                 static_cast<float>(config::kCameraGain));
  }
  if (supported.count(libcamera::controls::Brightness.id())) {
    controls.set(libcamera::controls::Brightness,
                 static_cast<float>(config::kCameraBrightness));
  }
  if (supported.count(libcamera::controls::Contrast.id())) {
    controls.set(libcamera::controls::Contrast,
                 static_cast<float>(config::kCameraContrast));
  }
  if (supported.count(libcamera::controls::Saturation.id())) {
    controls.set(libcamera::controls::Saturation,
                 static_cast<float>(config::kCameraSaturation));
  }
  if (config::kCameraWhiteBalance > 0 &&
      supported.count(libcamera::controls::ColourTemperature.id())) {
    controls.set(libcamera::controls::ColourTemperature, config::kCameraWhiteBalance);
  }
}

bool CameraCapture::Impl::queueRequest(libcamera::Request* request) {
  if (!request || !camera_ || !running_) return false;
  request->reuse(libcamera::Request::ReuseBuffers);
  applyControls(request);
  const int ret = camera_->queueRequest(request);
  if (ret < 0) {
    std::fprintf(stderr, "ballalgo libcamera queueRequest failed: %d\n", ret);
    return false;
  }
  return true;
}

void CameraCapture::Impl::onRequestCompleted(libcamera::Request* request) {
  std::lock_guard<std::mutex> lock(mutex_);
  completedRequests_.push_back(request);
  completedCv_.notify_one();
}

bool CameraCapture::Impl::copyRequestToMat(libcamera::Request* request, cv::Mat& bgrOut) {
  if (!request || request->status() == libcamera::Request::RequestCancelled) return false;

  const auto bufferIt = request->buffers().find(stream_);
  if (bufferIt == request->buffers().end() || !bufferIt->second) return false;

  const auto mappedIt = bufferMappings_.find(bufferIt->second);
  if (mappedIt == bufferMappings_.end() || mappedIt->second.empty()) return false;
  const auto& mappedPlanes = mappedIt->second;

  if (pixelFormat_ == libcamera::formats::BGR888) {
    const cv::Mat wrapped(height_, width_, CV_8UC3, mappedPlanes[0].data, stride_);
    wrapped.copyTo(bgrOut);
    return true;
  }

  if (pixelFormat_ == libcamera::formats::RGB888) {
    const cv::Mat wrapped(height_, width_, CV_8UC3, mappedPlanes[0].data, stride_);
    cv::cvtColor(wrapped, bgrOut, cv::COLOR_RGB2BGR);
    return true;
  }

  std::fprintf(stderr, "ballalgo libcamera unsupported configured pixel format: %s\n",
               pixelFormat_.toString().c_str());
  return false;
}

bool CameraCapture::Impl::grab(cv::Mat& bgrOut) {
  if (!camera_) return false;

  libcamera::Request* request = nullptr;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    completedCv_.wait(lock, [&] { return !completedRequests_.empty() || !running_; });
    if (!running_ && completedRequests_.empty()) return false;

    request = completedRequests_.front();
    completedRequests_.pop_front();
  }

  const bool ok = copyRequestToMat(request, bgrOut);
  if (running_ && request && request->status() != libcamera::Request::RequestCancelled &&
      !queueRequest(request)) {
    running_ = false;
    completedCv_.notify_all();
  }

  return ok;
}

void CameraCapture::Impl::close() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    completedRequests_.clear();
  }
  completedCv_.notify_all();

  if (camera_) camera_->requestCompleted.disconnect(this);
  if (camera_) camera_->stop();

  requests_.clear();
  bufferMappings_.clear();

  if (allocator_ && stream_) allocator_->free(stream_);
  allocator_.reset();
  config_.reset();
  stream_ = nullptr;
  width_ = 0;
  height_ = 0;
  stride_ = 0;
  pixelFormat_ = {};

  if (camera_) camera_->release();
  camera_.reset();

  if (manager_) manager_->stop();
  manager_.reset();
}

CameraCapture::CameraCapture() : impl_(std::make_unique<Impl>()) {}

CameraCapture::~CameraCapture() = default;

bool CameraCapture::open() { return impl_->open(); }

bool CameraCapture::grab(cv::Mat& bgrOut) { return impl_->grab(bgrOut); }

void CameraCapture::close() { impl_->close(); }

}  // namespace ballalgo
