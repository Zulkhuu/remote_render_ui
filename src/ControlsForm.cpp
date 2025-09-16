// Copyright (c) 2022 Graphcore Ltd. All rights reserved.

#include "ControlsForm.hpp"
#include "custom_widgets/rotator.hpp"
#include <nanogui/graph.h>

#include <PacketSerialisation.h>
#include <cereal/types/string.hpp>

#include <boost/log/trivial.hpp>

#include <iomanip>
#include <fstream>

std::uint32_t convertSampleValue(float value) {
  // Maximum of 16 otherwise latency will be too high:
  std::uint32_t sampleCount = value * 16;
  // Must be at least 1:
  return std::max(sampleCount, 1u);
}

ControlsForm::ControlsForm(nanogui::Screen* screen,
                           PacketMuxer& sender,
                           PacketDemuxer& receiver,
                           VideoPreviewWindow* videoPreview)
    : nanogui::FormHelper(screen),
      hdrHeader(packets::HdrHeader{0,0,0}),
      saveButton(nullptr),
      preview(videoPreview)
{
  window = add_window(nanogui::Vector2i(10, 10), "Control");

  // Scene controls
  add_group("Scene Parameters");
  auto* rotationWheel = new Rotator(window);
  rotationWheel->set_callback([&](float value) {
    float angle = value/(2.f*M_PI) * 360.f;
    serialise(sender, "env_rotation", angle);
  });
  add_widget("Env NIF Rotation", rotationWheel);

  auto* rotationWheel2 = new Rotator(window);
  rotationWheel2->set_callback([&](float value) {
    float angle = value/(2.f*M_PI) * 360.f;
    serialise(sender, "env_rotation_2", angle);
  });
  add_widget("Env NIF Rotation", rotationWheel2);


  // Camera controls
  add_group("Camera Parameters");
  fovSlider = new nanogui::Slider(window);
  fovSlider->set_fixed_width(250);
  fovSlider->set_callback([&](float value) {
    serialise(sender, "fov",value * 180.f);
  });
  fovSlider->set_value(60.f / 180.f);
  fovSlider->callback()(fovSlider->value());
  add_widget("Field of View", fovSlider);

  // Subscribe to FOV updates from the server (on start-up the server can decide the initial value):
  subs["fov"] = receiver.subscribe("fov", [this](const ComPacket::ConstSharedPacket& packet) {
    float fovRadians = 0.f;
    deserialise(packet, fovRadians);
    BOOST_LOG_TRIVIAL(trace) << "Received FOV update: " << fovRadians;
    fovSlider->set_value(fovRadians / (M_PI));
  });

  // Render controls:
  // add_group("Variable Parameters");

  auto* XSlider = new nanogui::Slider(window);
  XSlider->set_fixed_width(250);
  XSlider->set_callback([&](float value) {
    serialise(sender, "X", value);
  });
  XSlider->set_value(0.35f);
  XSlider->callback()(XSlider->value());
  add_widget("X", XSlider);

  auto* YSlider = new nanogui::Slider(window);
  YSlider->set_fixed_width(250);
  YSlider->set_callback([&](float value) {
    serialise(sender, "Y", value);
  });
  YSlider->set_value(0.5f);
  YSlider->callback()(YSlider->value());
  add_widget("Y", YSlider);

  auto* ZSlider = new nanogui::Slider(window);
  ZSlider->set_fixed_width(250);
  ZSlider->set_callback([&](float value) {
    serialise(sender, "Z", value);
  });
  ZSlider->set_value(0.5f);
  ZSlider->callback()(ZSlider->value());
  add_widget("Z", ZSlider);
  
  // Info/stats
  add_group("Workloads");

  auto l4_hist = new nanogui::Graph(window);
  l4_hist->set_caption(" ");
  add_widget("L4 routers", l4_hist);
  auto l3_hist = new nanogui::Graph(window);
  l3_hist->set_caption(" ");
  add_widget("L3 routers", l3_hist);
  auto l2_hist = new nanogui::Graph(window);
  l2_hist->set_caption(" ");
  add_widget("L2 routers", l2_hist);
  auto l1_hist = new nanogui::Graph(window);
  l1_hist->set_caption(" ");
  add_widget("L1 routers", l1_hist);
  auto l0_hist = new nanogui::Graph(window);
  l0_hist->set_caption(" ");
  add_widget("L0 routers", l0_hist);
  auto hist = new nanogui::Graph(window);
  hist->set_caption(" ");
  add_widget("Ray Tracers", hist);
  add_group("Info/Stats");

  subs["tile_histogram"] = receiver.subscribe("tile_histogram", [hist, l0_hist, l1_hist, l2_hist, l3_hist, l4_hist](const ComPacket::ConstSharedPacket& packet) {
    std::vector<std::uint32_t> data;
    deserialise(packet, data);
    std::vector<float> dataf;
    dataf.reserve(data.size());
    const unsigned max_rays = data.back();
    const float scale = 1.f / max_rays;
    for (const auto& v : data) {
      dataf.push_back(v * scale);
    }
    auto slice = [&](std::size_t off, std::size_t count) -> std::vector<float> {
      const std::size_t n = dataf.size();
      const std::size_t take = (off < n) ? std::min(count, n - off) : 0;
      return std::vector<float>(dataf.begin() + off, dataf.begin() + off + take);
    };

    constexpr std::size_t kNumRayTracerTiles  = 1024;
    constexpr std::size_t kChildrenPerRouter = 4;
    constexpr std::size_t kNumL0RouterTiles = kNumRayTracerTiles / kChildrenPerRouter; // 256
    constexpr std::size_t kNumL1RouterTiles = kNumL0RouterTiles / kChildrenPerRouter; // 64
    constexpr std::size_t kNumL2RouterTiles = kNumL1RouterTiles / kChildrenPerRouter; // 16;
    constexpr std::size_t kNumL3RouterTiles = kNumL2RouterTiles / kChildrenPerRouter; // 4
    constexpr std::size_t kNumL4RouterTiles = kNumL3RouterTiles / kChildrenPerRouter; 

    constexpr std::size_t l0_base = kNumRayTracerTiles;
    constexpr std::size_t l1_base = l0_base + kNumL0RouterTiles;
    constexpr std::size_t l2_base = l1_base + kNumL1RouterTiles;
    constexpr std::size_t l3_base = l2_base + kNumL2RouterTiles;
    constexpr std::size_t l4_base = l3_base + kNumL3RouterTiles;

    std::vector<float> rtSlice = slice(0, kNumRayTracerTiles);
    rtSlice.resize(kNumRayTracerTiles, 0.f);

    std::vector<float> l0Src = slice(l0_base, kNumL0RouterTiles);  // 256
    std::vector<float> l1Src = slice(l1_base, kNumL1RouterTiles);  // 64
    std::vector<float> l2Src = slice(l2_base, kNumL2RouterTiles);  // 16
    std::vector<float> l3Src = slice(l3_base, kNumL3RouterTiles);  // 4
    std::vector<float> l4Src = slice(l4_base, 1);  // 1

    std::vector<float> l0Expanded(kNumRayTracerTiles, 0.f);
    for (std::size_t i = 0; i < l0Src.size(); ++i) {
      const std::size_t idx = 256 + i * 2;      // 2, 6, 10, ... 1022
      if (idx < l0Expanded.size()) l0Expanded[idx] = l0Src[i];
    }

    std::vector<float> l1Expanded(kNumRayTracerTiles, 0.f); 
    for (std::size_t i = 0; i < l1Src.size(); ++i) {
      const std::size_t idx = 256 + 128 + i * 4;    // 10, 26, 42, ... 1018
      if (idx < l1Expanded.size()) l1Expanded[idx] = l1Src[i];
    }

    std::vector<float> l2Expanded(kNumRayTracerTiles, 0.f);  
    for (std::size_t i = 0; i < l2Src.size(); ++i) {
      const std::size_t idx = 256 + 128 + 64 + i * 8;    // 10, 26, 42, ... 1018
      if (idx < l2Expanded.size()) l2Expanded[idx] = l2Src[i];
    }

    std::vector<float> l3Expanded(kNumRayTracerTiles, 0.f);  
    for (std::size_t i = 0; i < l3Src.size(); ++i) {
      const std::size_t idx = 256 + 128 + 64 + 32 + i * 16;    // 10, 26, 42, ... 1018
      if (idx < l3Expanded.size()) l3Expanded[idx] = l3Src[i];
    }
    std::vector<float> l4Expanded(kNumRayTracerTiles, 0.f);  
    l4Expanded[512] = l4Src[0];

    std::stringstream ss;
    ss << "Capacity: " << max_rays;
    l4_hist->set_header(ss.str());
    hist->set_values(rtSlice);
    l0_hist->set_values(l0Expanded);
    l1_hist->set_values(l1Expanded);
    l2_hist->set_values(l2Expanded);
    l3_hist->set_values(l3Expanded);
    l4_hist->set_values(l4Expanded);
  });

  // bitRateText = new nanogui::TextBox(window, "-");
  // bitRateText->set_editable(false);
  // bitRateText->set_units("Mbps");
  // bitRateText->set_alignment(nanogui::TextBox::Alignment::Right);
  // add_widget("Video rate:", bitRateText);

  frameRateText = new nanogui::TextBox(window, "-");
  frameRateText->set_editable(false);
  frameRateText->set_units("Updates/sec");
  frameRateText->set_alignment(nanogui::TextBox::Alignment::Right);
  add_widget("Refresh rate:", frameRateText);

  // Status/stop button:
  add_group("Render Mode");

  modeChooser = new nanogui::ComboBox(window, {"rgb", "depth"});
  modeChooser->set_enabled(true);
  modeChooser->set_side(nanogui::Popup::Side::Right);
  modeChooser->set_tooltip("Select rendering mode: {rgb, depth}");
  modeChooser->set_callback([&](int index) {
    auto modeString = modeChooser->items()[index];
    BOOST_LOG_TRIVIAL(debug) << "Sending new mode: " << modeString;
    serialise(sender, "mode", modeString);
  });
  modeChooser->set_font_size(16);
  add_widget("Choose render mode: ", modeChooser);

  add_button("Stop", [screen, &sender]() {
    serialise(sender, "stop", true);
    screen->set_visible(false);
  })->set_tooltip("Stop the remote application.");
}

void ControlsForm::set_position(const nanogui::Vector2i& pos) {
  window->set_position(pos);
}

void ControlsForm::savePfm(const std::string& fileName) {
  if (!hdrBuffer.empty()) {
    // Do not want to save a partially received image:
    std::lock_guard<std::mutex> lock(hdrBufferMutex);

    // Last packet so write a PFM file:
    std::ofstream f(fileName, std::ios::binary);
    f << "PF\n";
    f << std::to_string(hdrHeader.width) << " ";
    f << std::to_string(hdrHeader.height) << "\n";
    f << "-1.0\n";
    for (auto r = hdrHeader.height - 1; r >= 0; --r) {
      auto rowStart = reinterpret_cast<char*>(hdrBuffer.data() + (r * hdrHeader.width * 3));
      f.write(rowStart, hdrHeader.width * 3 * sizeof(float));
    }
  }
}
