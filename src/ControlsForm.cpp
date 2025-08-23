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
  add_group("Info/Stats");

  auto hist = new nanogui::Graph(window);
  hist->set_caption("Splats per tile");
  add_widget("Workload Balance", hist);

  subs["tile_histogram"] = receiver.subscribe("tile_histogram", [hist](const ComPacket::ConstSharedPacket& packet) {
    std::vector<std::uint32_t> data;
    deserialise(packet, data);
    std::vector<float> dataf;
    dataf.reserve(data.size());

    std::uint32_t max = 0.f;
    for (const auto& v : data) {
      if (v > max) { max = v; }
    }

    const float scale = 1.f / max;
    for (const auto& v : data) {
      dataf.push_back(v * scale);
    }
    std::stringstream ss;
    ss << "max tile: " << max;
    hist->set_header(ss.str());
    hist->set_values(dataf);
  });

  bitRateText = new nanogui::TextBox(window, "-");
  bitRateText->set_editable(false);
  bitRateText->set_units("Mbps");
  bitRateText->set_alignment(nanogui::TextBox::Alignment::Right);
  add_widget("Video rate:", bitRateText);

  frameRateText = new nanogui::TextBox(window, "-");
  frameRateText->set_editable(false);
  frameRateText->set_units("Frames/sec");
  frameRateText->set_alignment(nanogui::TextBox::Alignment::Right);
  add_widget("Frame rate:", frameRateText);

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
