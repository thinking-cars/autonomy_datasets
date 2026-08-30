// Copyright Thinking Cars GmbH
// SPDX-License-Identifier: Apache-2.0

#include "autonomy_datasets_rviz_plugins/playback_panel.hpp"

#include <memory>

#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QRegularExpression>
#include <QStringList>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace autonomy_datasets_rviz_plugins {

namespace {

using RequestSamples = autonomy_datasets_msgs::srv::RequestSamples;

//! Service of a dataset node that is started with its default node name
const char* const kDefaultServiceName = "/datasets/request_samples";

//! Interval in milliseconds at which the panel polls whether the service is available
constexpr int kAvailabilityPollIntervalMs = 500;

//! Largest step size offered, chosen so that a step can cross a whole scene at once
constexpr int kMaximumStepSize = 100000;

}  // namespace

PlaybackPanel::PlaybackPanel(QWidget* parent) : rviz_common::Panel(parent) { setupUi(); }

PlaybackPanel::~PlaybackPanel() {
  // Drops the pending request together with its callback, which would otherwise report into a
  // panel that no longer exists. RViz spins its node from the GUI thread that also destroys the
  // panel, so no response can be in the middle of being delivered while this runs.
  client_.reset();
}

void PlaybackPanel::setupUi() {
  // NOLINTBEGIN(cppcoreguidelines-owning-memory) Qt widgets are owned by their parent widget
  service_name_edit_ = new QLineEdit(QString::fromUtf8(kDefaultServiceName), this);
  service_name_edit_->setToolTip(
      tr("Name of the 'request_samples' service of the dataset node, as reported by 'ros2 service list'"));

  auto* service_layout = new QFormLayout();
  service_layout->addRow(tr("Service"), service_name_edit_);

  pause_button_ = new QPushButton(tr("Pause"), this);
  pause_button_->setToolTip(tr("Stop the playback at the current sample, without publishing another one"));
  play_button_ = new QPushButton(tr("Play to End"), this);
  play_button_->setToolTip(
      tr("Publish all remaining samples of the dataset.\n"
         "The dataset node answers this request only once the dataset ends, so the playback cannot be paused again "
         "until then."));

  auto* transport_layout = new QHBoxLayout();
  transport_layout->addWidget(pause_button_);
  transport_layout->addWidget(play_button_);

  step_size_spin_ = new QSpinBox(this);
  step_size_spin_->setRange(1, kMaximumStepSize);
  step_size_spin_->setValue(1);
  step_size_spin_->setToolTip(tr("Number of samples a step publishes"));
  step_button_ = new QPushButton(tr("Step"), this);
  step_button_->setToolTip(tr("Publish the next samples, whichever they are"));

  auto* step_layout = new QHBoxLayout();
  step_layout->addWidget(step_size_spin_);
  step_layout->addWidget(new QLabel(tr("sample(s)"), this));
  step_layout->addStretch();
  step_layout->addWidget(step_button_);

  sample_ids_edit_ = new QLineEdit(this);
  sample_ids_edit_->setPlaceholderText(tr("e.g. 120 or 10, 20, 30"));
  sample_ids_edit_->setToolTip(
      tr("IDs of the samples to publish, skipping every sample in between.\n"
         "The playback is forward-only, so it cannot go back to a sample it has already passed."));
  skip_button_ = new QPushButton(tr("Skip To"), this);
  skip_button_->setToolTip(tr("Skip ahead to the entered samples"));

  auto* skip_layout = new QHBoxLayout();
  skip_layout->addWidget(sample_ids_edit_);
  skip_layout->addWidget(skip_button_);

  auto* separator = new QFrame(this);
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);

  position_label_ = new QLabel(tr("No sample published yet"), this);
  status_label_ = new QLabel(tr("Not connected to a dataset node yet"), this);
  status_label_->setWordWrap(true);

  auto* layout = new QVBoxLayout();
  layout->addLayout(service_layout);
  layout->addLayout(transport_layout);
  layout->addLayout(step_layout);
  layout->addLayout(skip_layout);
  layout->addWidget(separator);
  layout->addWidget(position_label_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  availability_timer_ = new QTimer(this);
  // NOLINTEND(cppcoreguidelines-owning-memory)

  connect(service_name_edit_, &QLineEdit::editingFinished, this, &PlaybackPanel::onServiceNameChanged);
  connect(pause_button_, &QPushButton::clicked, this, &PlaybackPanel::onPauseClicked);
  connect(play_button_, &QPushButton::clicked, this, &PlaybackPanel::onPlayClicked);
  connect(step_button_, &QPushButton::clicked, this, &PlaybackPanel::onStepClicked);
  connect(skip_button_, &QPushButton::clicked, this, &PlaybackPanel::onSkipClicked);
  connect(sample_ids_edit_, &QLineEdit::returnPressed, this, &PlaybackPanel::onSkipClicked);
  connect(availability_timer_, &QTimer::timeout, this, &PlaybackPanel::updateAvailability);
  connect(step_size_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, &PlaybackPanel::configChanged);

  // The response of a request may reach the panel on whichever thread the executor of RViz runs
  // on, so the widgets are updated through a queued connection instead of from the callback.
  connect(this, &PlaybackPanel::requestFinished, this, &PlaybackPanel::applyRequestResult, Qt::QueuedConnection);

  updateControlState();
}

void PlaybackPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  createClient();
  availability_timer_->start(kAvailabilityPollIntervalMs);
}

void PlaybackPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);

  QString service_name;
  if (config.mapGetString("ServiceName", &service_name) && !service_name.isEmpty()) {
    service_name_edit_->setText(service_name);
  }
  float step_size = 0.0F;
  if (config.mapGetFloat("StepSize", &step_size) && step_size >= 1.0F) {
    step_size_spin_->setValue(static_cast<int>(step_size));
  }
  if (node_ != nullptr) {
    createClient();
  }
}

void PlaybackPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("ServiceName", service_name_edit_->text());
  config.mapSetValue("StepSize", step_size_spin_->value());
}

void PlaybackPanel::createClient() {
  const QString service_name = service_name_edit_->text().trimmed();
  if (node_ == nullptr || service_name.isEmpty()) {
    client_.reset();
    updateAvailability();
    return;
  }
  client_ = node_->create_client<RequestSamples>(service_name.toStdString());
  service_available_ = false;
  updateAvailability();
}

void PlaybackPanel::onServiceNameChanged() {
  createClient();
  Q_EMIT configChanged();
}

void PlaybackPanel::updateAvailability() {
  const bool available = client_ != nullptr && client_->service_is_ready();
  if (available == service_available_) {
    updateControlState();
    return;
  }
  service_available_ = available;

  if (!available && request_in_flight_) {
    // The node answering the request is gone, so its response will never arrive
    request_in_flight_ = false;
    status_label_->setText(tr("The dataset node disappeared while it was processing the request"));
  } else if (!available) {
    status_label_->setText(tr("Waiting for service '%1'").arg(service_name_edit_->text().trimmed()));
  } else if (!request_in_flight_) {
    status_label_->setText(tr("Connected to '%1'").arg(service_name_edit_->text().trimmed()));
  }
  updateControlState();
}

void PlaybackPanel::updateControlState() {
  const bool ready = service_available_ && !request_in_flight_;
  pause_button_->setEnabled(ready);
  play_button_->setEnabled(ready);
  step_button_->setEnabled(ready);
  skip_button_->setEnabled(ready);
  step_size_spin_->setEnabled(ready);
  sample_ids_edit_->setEnabled(ready);
}

void PlaybackPanel::onPauseClicked() {
  // A request for no sample at all takes control of the playback without publishing anything,
  // which leaves the playback waiting for the next request
  sendRequest(RequestSamples::Request::MODE_NEXT_SAMPLES, 0, {}, tr("Pausing playback..."));
}

void PlaybackPanel::onStepClicked() {
  const auto num_samples = static_cast<uint64_t>(step_size_spin_->value());
  sendRequest(RequestSamples::Request::MODE_NEXT_SAMPLES, num_samples, {},
              tr("Publishing the next %1 sample(s)...").arg(static_cast<qulonglong>(num_samples)));
}

void PlaybackPanel::onSkipClicked() {
  QString error;
  const std::vector<uint64_t> sample_ids = parseSampleIds(&error);
  if (!error.isEmpty()) {
    status_label_->setText(error);
    return;
  }
  if (sample_ids.empty()) {
    status_label_->setText(tr("Enter the ID of the sample to skip to"));
    return;
  }
  sendRequest(RequestSamples::Request::MODE_SAMPLE_IDS, 0, sample_ids,
              tr("Skipping ahead to %1 sample(s)...").arg(static_cast<qulonglong>(sample_ids.size())));
}

void PlaybackPanel::onPlayClicked() {
  sendRequest(RequestSamples::Request::MODE_ALL_SAMPLES, 0, {}, tr("Publishing all remaining samples..."));
}

std::vector<uint64_t> PlaybackPanel::parseSampleIds(QString* error) const {
  std::vector<uint64_t> sample_ids;
  const QStringList entries = sample_ids_edit_->text().split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts);
  for (const QString& entry : entries) {
    bool valid = false;
    const qulonglong sample_id = entry.toULongLong(&valid);
    if (!valid) {
      *error = tr("'%1' is not a sample ID").arg(entry);
      return {};
    }
    sample_ids.push_back(static_cast<uint64_t>(sample_id));
  }
  return sample_ids;
}

void PlaybackPanel::sendRequest(uint8_t mode,
                                uint64_t num_samples,
                                const std::vector<uint64_t>& sample_ids,
                                const QString& pending_status) {
  if (client_ == nullptr || !client_->service_is_ready()) {
    status_label_->setText(tr("Service '%1' is not available").arg(service_name_edit_->text().trimmed()));
    return;
  }

  auto request = std::make_shared<RequestSamples::Request>();
  request->mode = mode;
  request->num_samples = num_samples;
  request->sample_ids = sample_ids;

  request_in_flight_ = true;
  status_label_->setText(pending_status);
  updateControlState();

  client_->async_send_request(request, [this](rclcpp::Client<RequestSamples>::SharedFuture future) {
    const auto response = future.get();
    QString status = QString::fromStdString(response->message);
    if (response->end_of_dataset) {
      status += tr("; the dataset has ended");
    }
    Q_EMIT requestFinished(describePosition(*response), status, response->success);
  });
}

QString PlaybackPanel::describePosition(const RequestSamples::Response& response) {
  if (response.published_sample_ids.empty()) {
    return {};
  }
  const uint64_t sample_id = response.published_sample_ids.back();
  if (response.published_scene_ids.size() != response.published_sample_ids.size()) {
    return tr("Sample %1").arg(static_cast<qulonglong>(sample_id));
  }
  return tr("Sample %1 of scene '%2'")
      .arg(static_cast<qulonglong>(sample_id))
      .arg(QString::fromStdString(response.published_scene_ids.back()));
}

void PlaybackPanel::applyRequestResult(const QString& position, const QString& status, bool success) {
  request_in_flight_ = false;
  if (!position.isEmpty()) {
    position_label_->setText(position);
  }
  status_label_->setText(success ? tr("OK - %1").arg(status) : tr("Failed - %1").arg(status));
  updateControlState();
}

}  // namespace autonomy_datasets_rviz_plugins

PLUGINLIB_EXPORT_CLASS(autonomy_datasets_rviz_plugins::PlaybackPanel, rviz_common::Panel)

// Compiles the code Qt generates for the signals and slots of the panel into this translation
// unit, instead of leaving it in one of its own that would have to be excluded from linting.
#include "autonomy_datasets_rviz_plugins/moc_playback_panel.cpp"
