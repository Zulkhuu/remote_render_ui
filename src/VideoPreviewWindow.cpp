// Copyright (c) 2022 Graphcore Ltd. All rights reserved.

#include "VideoPreviewWindow.hpp"

#include <PacketComms.h>

#include <boost/log/trivial.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

VideoPreviewWindow::VideoPreviewWindow(
    nanogui::Screen* screen,
    const std::string& title,
    PacketDemuxer& receiver)
    : nanogui::Window(screen, title),
      videoClient(std::make_unique<VideoClient>(receiver, "render_preview")),
      texture(nullptr),
      mbps(0.0),
      m_lastFrameTime(std::chrono::steady_clock::now()),
      fps(0.f),
      upscale(1.2f),
      newFrameDecoded(false),
      runDecoderThread(true),
      showRawPixelValues(false) {
  using namespace nanogui;
  using namespace std::chrono_literals;
  bool videoOk = videoClient->initialiseVideoStream(5s);

  if (videoOk) {
    // Allocate a buffer to store the decoded and converted images:
    auto w = videoClient->getFrameWidth();
    auto h = videoClient->getFrameHeight();
    int wu = int(w*upscale);
    int hu = int(h*upscale);

    // Create the texture first because internally nanogui will create
    // one with the preferred format and we need to know how to allcoate
    // the buffers:
    texture = new Texture(
        Texture::PixelFormat::RGB,
        Texture::ComponentFormat::UInt8,
        Vector2i(wu, hu),
        Texture::InterpolationMode::Trilinear,
        Texture::InterpolationMode::Nearest);
    const auto ch = texture->channels();
    BOOST_LOG_TRIVIAL(trace) << "Created texture with "
                             << texture->channels() << " channels, "
                             << "data ptr: " << (void*)bgrBuffer.data();
    if (!(ch == 3 || ch == 4)) {
      throw std::logic_error("Texture returned has an unsupported number of texture channels.");
    }

    bgrBuffer.resize(wu * hu * ch);
    this->set_size(Vector2i(wu, hu));
    this->set_layout(new GroupLayout(0));
    imageView = new ImageView(this);

    for (auto c = 0; c < bgrBuffer.size(); c += ch) {
      bgrBuffer[c + 0] = 255;
      bgrBuffer[c + 1] = 0;
      bgrBuffer[c + 2] = 0;
      if (ch == 4) {
        bgrBuffer[c + 3] = 128;
      }
    }

    texture->upload(bgrBuffer.data());

    imageView->set_size(Vector2i(wu, hu));
    imageView->set_image(texture);
    imageView->center();
    imageView->set_pixel_callback(
        [this](const Vector2i& pos, char** out, size_t size) {
          // The information provided by this callback is used to
          // display pixel values at high magnification:
          auto w = videoClient->getFrameWidth();
          if (showRawPixelValues && !rawBuffer.empty()) {
            // If we have the raw HDR data available then use that
            // for the pixel labels instead:
            std::size_t index = (pos.x() + w * pos.y()) * 3;
            for (int c = 0; c < 3; ++c) {
              float value = rawBuffer[index + c];
              snprintf(out[c], size, "%.2f", value);
            }
          } else {
            std::size_t index = (pos.x() + w * pos.y()) * texture->channels();
            for (int c = 0; c < texture->channels(); ++c) {
              uint8_t value = bgrBuffer[index + c];
              snprintf(out[c], size, "%i", (int)value);
            }
          }
        });

    BOOST_LOG_TRIVIAL(info) << "Succesfully initialised video stream.";
    startDecodeThread();

  } else {
    BOOST_LOG_TRIVIAL(warning) << "Failed to initialise video stream.";
  }
}

VideoPreviewWindow::~VideoPreviewWindow() {
  stopDecodeThread();
}

void VideoPreviewWindow::startDecodeThread() {
  // Thread just decodes video frames as fast as it can:
  videoDecodeThread.reset(new std::thread([&]() {
    BOOST_LOG_TRIVIAL(debug) << "Video decode thread launched.";
    if (videoClient == nullptr) {
      BOOST_LOG_TRIVIAL(debug) << "Video client must be initialised before decoding.";
      throw std::logic_error("No VideoClient object available.");
    }

    while (runDecoderThread) {
      decodeVideoFrame();
    }
  }));
}

void VideoPreviewWindow::stopDecodeThread() {
  runDecoderThread = false;
  if (videoDecodeThread) {
    try {
      videoDecodeThread->join();
      BOOST_LOG_TRIVIAL(debug) << "Video decode thread joined successfully.";
      videoDecodeThread.reset();
    } catch (std::system_error& e) {
      BOOST_LOG_TRIVIAL(warning) << "Video decode thread could not be joined.";
    }
  }
}

/// Decode a video frame into the buffer.
void VideoPreviewWindow::decodeVideoFrame() {
  newFrameDecoded = videoClient->receiveVideoFrame(
      [this](LibAvCapture& stream) {
        BOOST_LOG_TRIVIAL(debug) << "Decoded video frame";
        auto w = stream.GetFrameWidth();
        auto h = stream.GetFrameHeight();
        if (texture != nullptr) {
          // Extract decoded data to the buffer:
          std::lock_guard<std::mutex> lock(bufferMutex);
          int ch = texture->channels();
          std::vector<uint8_t> tmpBuffer(w * h * ch);
          

          if (ch == 3) {
            stream.ExtractRgbImage(tmpBuffer.data(), w * ch);
          } else if (ch == 4) {
            stream.ExtractRgbaImage(tmpBuffer.data(), w * ch);
          } else {
            throw std::runtime_error("Unsupported number of texture channels");
          }

          // --- Resize using OpenCV ---
          cv::Mat inputImg(h, w, (ch == 3) ? CV_8UC3 : CV_8UC4, tmpBuffer.data());
          cv::Mat upsampledImg;
          cv::resize(inputImg, upsampledImg, cv::Size(), upscale, upscale, cv::INTER_LINEAR);
          
          // Resize bgrBuffer to fit upsampled image
          bgrBuffer.resize(upsampledImg.total() * ch);
          std::memcpy(bgrBuffer.data(), upsampledImg.data, bgrBuffer.size());
          // After resizing
          if (texture->size() != nanogui::Vector2i(upsampledImg.cols, upsampledImg.rows)) {
            texture->resize(nanogui::Vector2i(upsampledImg.cols, upsampledImg.rows));
            this->set_size(nanogui::Vector2i(upsampledImg.cols, upsampledImg.rows));
            imageView->set_size(nanogui::Vector2i(upsampledImg.cols, upsampledImg.rows));
          }
        }
      });
  if (newFrameDecoded) {
    double bps = videoClient->computeVideoBandwidthConsumed();
    auto imbps = bps / (1024.0 * 1024.0);
    mbps = (0.9f * mbps) + (.1f * imbps);
    BOOST_LOG_TRIVIAL(trace) << "Video bit-rate instantaneous: " << imbps << " Mbps" << std::endl;
    BOOST_LOG_TRIVIAL(debug) << "Video bit-rate filtered: " << mbps << " Mbps" << std::endl;

    // Calculate instantaneous frame rate:
    auto newFrameTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(newFrameTime - m_lastFrameTime).count();
    auto ifps = 1000.0 / (std::max(elapsed, 1L));
    fps = (0.9f * fps) + (.1f * ifps);
    BOOST_LOG_TRIVIAL(trace) << "Frame rate instantaneous: " << ifps << " Fps" << std::endl;
    BOOST_LOG_TRIVIAL(debug) << "Frame rate filtered: " << fps << " Fps" << std::endl;
    m_lastFrameTime = newFrameTime;
  }
}

void VideoPreviewWindow::draw(NVGcontext* ctx) {
  // Upload latest buffer contents to video texture:
  if (texture != nullptr) {
    std::lock_guard<std::mutex> lock(bufferMutex);
    texture->upload(bgrBuffer.data());
  }

  nanogui::Window::draw(ctx);
}
